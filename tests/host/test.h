/*
 * Minimal unit test framework for host-side pure-logic tests.
 *
 * Usage:
 *   #include "test.h"
 *   static void my_test(void) { ASSERT_EQ_INT(1 + 1, 2); }
 *   int main(void) {
 *     TEST_BEGIN();
 *     TEST_CASE(my_test);
 *     TEST_END();
 *   }
 */
#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int test_failures = 0;
static int test_count = 0;
static const char *current_test = "(none)";

#define TEST_BEGIN()                                                          \
  do {                                                                        \
    test_failures = 0;                                                        \
    test_count = 0;                                                           \
    printf("=== %s ===\n", __FILE__);                                         \
  } while (0)

#define TEST_CASE(fn)                                                         \
  do {                                                                        \
    current_test = #fn;                                                       \
    test_count++;                                                             \
    int before = test_failures;                                               \
    fn();                                                                     \
    if (test_failures == before) printf("  PASS %s\n", #fn);                  \
  } while (0)

#define TEST_END()                                                            \
  do {                                                                        \
    printf("--- %d tests, %d failures ---\n", test_count, test_failures);    \
    return test_failures ? 1 : 0;                                             \
  } while (0)

#define TEST_FAIL(fmt, ...)                                                   \
  do {                                                                        \
    test_failures++;                                                          \
    fprintf(stderr, "  FAIL [%s:%d in %s] " fmt "\n", __FILE__, __LINE__,     \
            current_test, ##__VA_ARGS__);                                     \
  } while (0)

#define ASSERT_TRUE(cond)                                                     \
  do {                                                                        \
    if (!(cond)) TEST_FAIL("expected true: %s", #cond);                       \
  } while (0)

#define ASSERT_FALSE(cond)                                                    \
  do {                                                                        \
    if (cond) TEST_FAIL("expected false: %s", #cond);                         \
  } while (0)

#define ASSERT_EQ_INT(actual, expected)                                       \
  do {                                                                        \
    long _a = (long)(actual), _e = (long)(expected);                          \
    if (_a != _e)                                                             \
      TEST_FAIL("expected %ld, got %ld (%s)", _e, _a, #actual);               \
  } while (0)

#define ASSERT_EQ_BYTE(actual, expected)                                      \
  do {                                                                        \
    unsigned _a = (unsigned)(actual) & 0xFFu;                                 \
    unsigned _e = (unsigned)(expected) & 0xFFu;                               \
    if (_a != _e)                                                             \
      TEST_FAIL("expected 0x%02X, got 0x%02X (%s)", _e, _a, #actual);         \
  } while (0)

#define ASSERT_NEAR_FLOAT(actual, expected, tol)                              \
  do {                                                                        \
    float _a = (float)(actual), _e = (float)(expected), _t = (float)(tol);    \
    if (fabsf(_a - _e) > _t)                                                  \
      TEST_FAIL("expected %f ±%f, got %f (%s)", _e, _t, _a, #actual);         \
  } while (0)

#endif /* TEST_H */
