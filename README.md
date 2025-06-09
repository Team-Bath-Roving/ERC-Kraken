# Kraken Firmware for ERC Rover

This project provides a Micro-ROS control node for a 6-DOF robotic arm and an additional two-motor drill system, using the STM32-based BTT Kraken. It integrates motor control, homing routines, and ROS 2 communication for real-time robotic applications.

## Features

* Controls 6 stepper-driven joints plus two drill motors.
* Micro-ROS integration for communication with a ROS 2 system.
* Homing routine via ROS 2 service call.
* Position command subscription via ROS 2 topic.
* Homing status publisher.
* Full motor enable/disable via ROS service.
* Emergency stop via hardware input or ROS topic.
* Manual homing support for semi-differential wrist.
* Configurable homing order and offset support with reversible cam logic.

---

## Kraken Control Board

The Kraken is a powerful 3D printer control board which has 4x 8A stepper drivers, 4x 4A stepper drivers and various outputs for fans, servos, endstops, switches etc.
![](https://teamgloomy.github.io/images/btt_kraken_pins.png)
[https://github.com/bigtreetech/BIGTREETECH-Kraken/tree/master](https://github.com/bigtreetech/BIGTREETECH-Kraken/tree/master)

---

## micro-ROS for PlatformIO

This library is used to communicate with the rest of the system:
[https://github.com/micro-ROS/micro\_ros\_platformio](https://github.com/micro-ROS/micro_ros_platformio)

### File Structure

* `motor.h`, `motor_control.h`: Encapsulate stepper motor behavior and control logic.
* `kraken_pinout.h`: Defines pin assignments for Kraken board.
* `configuration.h`: Optional global parameters (e.g., timing, step resolution).
* `main.cpp`: ROS 2 integration, homing logic, and execution loop.

---

## Setup

### Dependencies

Run the following to ensure you have the correct packages:

```bash
sudo apt update
sudo apt install python3-venv cmake gcc g++
```

Install the PlatformIO extension for VSCode.
**Note:** micro-ROS PlatformIO currently only builds reliably on Ubuntu / Debian / Linux Mint. Windows builds are not supported.

---

## Usage

1. Flash the firmware (compile and upload `firmware.bin` by placing it on an SD card and inserting it into the Kraken, then power cycle).
2. Connect to the host machine via USB (CAN support planned).
3. Send position commands via the `joint_positions` topic.
4. Call the `home_all` service before commanding motion.

---

## ROS 2 Interfaces

### Topics

#### `joint_positions` (Subscription)

* **Type:** `std_msgs/msg/Int32MultiArray`
* **Description:** Target positions (in steps) for the 6 joints.
* **Expected Length:** 6-element array.

#### `homing_done` (Publisher)

* **Type:** `std_msgs/msg/Bool`
* **Description:** Publishes `true` once all joints and drills are homed.

#### `estop` (Subscription)

* **Type:** `std_msgs/msg/Bool`
* **Description:** Emergency stop override. If `true`, all motors are disabled.

---

### Services

#### `home_all`

* **Type:** `std_srvs/srv/Trigger`
* **Description:** Initiates the homing sequence for all joints and drills in a configurable order with optional home offsets.

#### `set_motors_enabled`

* **Type:** `std_srvs/srv/SetBool`
* **Description:** Enables or disables all motors via a service call.

  * `true`: Enable all motors.
  * `false`: Disable all motors.

#### `manual_home_wrist`

* **Type:** `std_srvs/srv/Trigger`
* **Description:** Temporarily disables the wrist motors to allow manual positioning. After a delay or signal, it resets their current position to zero.

---

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

### Enable Motors

```bash
ros2 service call /set_motors_enabled std_srvs/srv/SetBool "{data: true}"
```

### Disable Motors

```bash
ros2 service call /set_motors_enabled std_srvs/srv/SetBool "{data: false}"
```

### Manual Home the Wrist

```bash
ros2 service call /manual_home_wrist std_srvs/srv/Trigger
```

### Trigger Emergency Stop from ROS

```bash
ros2 topic pub /estop std_msgs/msg/Bool "{data: true}"
```

---

## Notes

* Homing **must** be performed before any movement command is accepted.
* A physical emergency stop input pin also disables all motors if triggered (active-low).
* Drill motors (`drill1`, `drill2`) are not yet commanded via ROS topics.
* Home offset and homing order are configurable in code.
* Wrist homing supports manual alignment mode for semi-differential drive configurations.

---

## License

SPDX-License-Identifier: GPL-3.0 License
