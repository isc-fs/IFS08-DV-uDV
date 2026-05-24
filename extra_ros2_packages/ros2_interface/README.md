# ros2_interface

Custom ROS2 interface package for the IFS08 uDV node, defining the
mission-orchestration service and action shared between the autonomy
stack and the STM32 driverless controller.

## Contents

- `srv/SetMission.srv` — autonomy → uDV; pre-loads the mission ID
- `action/StartMission.action` — autonomy → uDV; starts the mission and
  streams `acceleration` / `steering` commands as feedback, finalizes
  with `finished` / `emergency` flags

## Schema source of truth

This package was reconstructed from firmware usage in
`Core/Src/ros_interface.cpp` and the atomic types declared in
`Core/Inc/ros_globals.h`. Field names and types are pinned by what the
firmware reads — changing them here will break the firmware build.

## Used by

- Firmware (this repo): baked into `libmicroros.a` via the micro-ROS
  library_generation workflow that consumes this folder.
- Autonomy stack: depends on this package to publish goals / consume
  feedback.

When this package needs to be consumed from another repo, copy the
folder or split it into its own repo and update the consumer.
