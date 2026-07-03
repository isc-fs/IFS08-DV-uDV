#!/bin/sh
# check_build_integrity.sh — static build-CONFIGURATION checks that the C++
# host unit tests cannot cover. The unit suites exercise runtime *logic*; they
# link a hand-picked subset of sources and therefore never notice when the
# real firmware build is mis-wired.
#
# Motivating bug: CMakeLists.txt drifted out of sync with the Makefile — the
# whole watchdog / AS-state-machine source set (safety_monitor.c, app_task.cpp,
# …) was missing from the CMake list, so the firmware failed to LINK with
#   undefined reference to `safety_arm' / `StartAppTask' / …
# Pure logic tests are blind to that. This script squashes the class.
#
# Portable POSIX sh + grep/sed/sort/comm — runs on macOS and Linux, needs no
# cross toolchain. Reports every category, exits non-zero if any fail.

set -u

# repo root = two levels up from this script (tests/host/ -> repo)
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
cd "$ROOT" || exit 2

MK=Makefile
CM=CMakeLists.txt
fail=0
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT INT TERM

# Pull the enumerated translation units (.c/.cpp/.s) out of a build file.
# A source line reduces to a bare path ending in .c/.cpp/.s (Makefile allows a
# trailing "\" continuation). Comment lines (#…), rule lines ($…, %…:) and the
# add_executable() line never reduce to that, so they are excluded.
extract() {
  sed -E 's/[[:space:]]+$//; s/[[:space:]]*\\$//; s/^[[:space:]]+//' "$1" \
    | grep -E '^[A-Za-z0-9_][A-Za-z0-9_/.-]*\.(c|cpp|s)$'
}

echo "build integrity @ $ROOT"

# ---- 1. Makefile <-> CMakeLists source-list parity (THE drift bug) ---------
extract "$MK" | sort -u > "$TMP/mk"
extract "$CM" | sort -u > "$TMP/cm"
only_mk=$(comm -23 "$TMP/mk" "$TMP/cm")
only_cm=$(comm -13 "$TMP/mk" "$TMP/cm")
if [ -n "$only_mk" ] || [ -n "$only_cm" ]; then
  echo "FAIL [1] Makefile and CMakeLists.txt source lists differ:"
  [ -n "$only_mk" ] && { echo "  only in Makefile (CMake build would NOT link these):";    echo "$only_mk" | sed 's/^/      /'; }
  [ -n "$only_cm" ] && { echo "  only in CMakeLists (Make build would NOT link these):";    echo "$only_cm" | sed 's/^/      /'; }
  fail=1
else
  echo "PASS [1] Makefile <-> CMakeLists source lists identical ($(wc -l < "$TMP/mk" | tr -d ' ') TUs)"
fi

# ---- 2. every referenced source actually exists on disk --------------------
missing=""
for f in $(cat "$TMP/mk" "$TMP/cm" | sort -u); do
  [ -f "$f" ] || missing="$missing $f"
done
if [ -n "$missing" ]; then
  echo "FAIL [2] build references source(s) that do not exist on disk:"
  for f in $missing; do echo "      $f"; done
  fail=1
else
  echo "PASS [2] all referenced sources exist on disk"
fi

# ---- 3. no orphan application sources (Core/Src present but unbuilt) --------
# A new Core/Src/*.c[pp] that nobody wired into the build links fine until
# something calls it, then fails exactly like the motivating bug.
ls Core/Src/*.c Core/Src/*.cpp 2>/dev/null | sort -u > "$TMP/disk"
grep -E '^Core/Src/' "$TMP/mk" | sort -u > "$TMP/mk_core"
orphans=$(comm -23 "$TMP/disk" "$TMP/mk_core")
if [ -n "$orphans" ]; then
  echo "FAIL [3] Core/Src source(s) on disk are not in the build:"
  echo "$orphans" | sed 's/^/      /'
  echo "         -> add to BOTH Makefile and CMakeLists, or delete the file"
  fail=1
else
  echo "PASS [3] no orphan Core/Src sources"
fi

# ---- 4. no duplicate source entries within either list ---------------------
dup_mk=$(extract "$MK" | sort | uniq -d)
dup_cm=$(extract "$CM" | sort | uniq -d)
if [ -n "$dup_mk" ] || [ -n "$dup_cm" ]; then
  echo "FAIL [4] duplicate source entries:"
  [ -n "$dup_mk" ] && echo "$dup_mk" | sed 's/^/      Makefile:   /'
  [ -n "$dup_cm" ] && echo "$dup_cm" | sed 's/^/      CMakeLists: /'
  fail=1
else
  echo "PASS [4] no duplicate source entries"
fi

# ---- 5. C/C++ linkage trap -------------------------------------------------
# A FreeRTOS task entry point (Start*Task) defined in a .cpp but created from C
# (freertos.c -> osThreadNew) must be extern "C"; otherwise its C++-mangled
# name leaves an undefined reference at link — same failure signature, subtler
# cause. Flag any .cpp that defines a Start*Task without extern "C".
bad=""
for f in Core/Src/*.cpp; do
  [ -f "$f" ] || continue
  if grep -Eq '\bStart[A-Za-z0-9_]*Task[[:space:]]*\(' "$f"; then
    grep -q 'extern "C"' "$f" || bad="$bad $f"
  fi
done
if [ -n "$bad" ]; then
  echo "FAIL [5] .cpp defines a Start*Task entry point without extern \"C\":"
  for f in $bad; do echo "      $f"; done
  fail=1
else
  echo "PASS [5] C++ task entry points are extern \"C\""
fi

if [ "$fail" -ne 0 ]; then
  echo "build integrity: FAILED"
  exit 1
fi
echo "build integrity: OK"
