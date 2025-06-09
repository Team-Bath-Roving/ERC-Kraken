# 6-DOF Robot Arm and Drill Micro-ROS Node

This project provides a Micro-ROS control node for a 6-DOF robotic arm and an additional two-motor drill system, using an STM32-based board (e.g., BTT Kraken) and TMC2160 drivers. It integrates motor control, homing routines, and ROS 2 communication for real-time robotic applications.

## Features

* Controls 6 stepper-driven joints plus two drill motors.
* Micro-ROS integration for communication with a ROS 2 system.
* Homing routine via ROS 2 service call.
* Position command subscription via ROS 2 topic.
* Homing status publisher.

## ROS 2 Interfaces

### Topics

#### `joint_positions` (subscription)

* **Type:** `std_msgs/msg/Int32MultiArray`
* **Description:** Target positions (in steps) for the 6 joints.
* **Expected Length:** 6-element array

#### `homing_done` (publisher)

* **Type:** `std_msgs/msg/Bool`
* **Description:** Publishes `true` once all joints and drills are homed.

### Services

#### `home_all` (Trigger service)

* **Type:** `std_srvs/srv/Trigger`
* **Description:** Initiates the homing sequence for all joints and drills.
* **Response:**

  * `success: true`
  * `message: "Homing complete."`


## File Structure

* `motor.h`, `motor_control.h`: Encapsulate stepper motor behavior and control logic.
* `kraken_pinout.h`: Defines pin assignments for Kraken board.
* `configuration.h`: Optional global parameters (e.g., timing, step resolution).
* `main.cpp`: ROS 2 integration, homing logic, and execution loop.

## Setup

### Dependencies

Ensure you have the following:

* PlatformIO installed in VSCode
* micro-ROS Arduino library
* Proper configuration of serial transport:

  ```cpp
  set_microros_serial_transports(Serial);
  ```

### PlatformIO `platformio.ini` Example

```ini
[env:kraken]
platform = ststm32
board = genericSTM32F407VET6
framework = arduino
monitor_speed = 115200
lib_deps =
  micro-ROS/micro_ros_arduino
  ...
```

## Usage

1. Flash the firmware to the microcontroller.
2. Connect the ROS 2 agent on your host machine via micro-ROS serial bridge.
3. Send position commands via the `joint_positions` topic.
4. Call the `home_all` service before commanding motion.

## Example ROS 2 Commands

### Home the Arm

```bash
ros2 service call /home_all std_srvs/srv/Trigger
```

### Send Position Commands

```bash
ros2 topic pub /joint_positions std_msgs/msg/Int32MultiArray "{data: [1000, 2000, 1500, 1000, 500, 0]}"
```

### Subscribe to Homing Status

```bash
ros2 topic echo /homing_done
```

## Notes

* Homing **must** be performed before any movement command is accepted.
* Drill motors (`drill1`, `drill2`) are treated the same as joints but not commanded via ROS topics currently.

## License

SPDX-License-Identifier: BSD-3-Clause
(C) 2025 Charlie Spencer

---

Let me know if you'd like a version with installation instructions for a specific board or ROS 2 agent setup.
