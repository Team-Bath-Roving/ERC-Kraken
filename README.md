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

### Subscribed Topics

#### `joint_positions` (`std_msgs/msg/Int32MultiArray`)
Target positions (in steps) for the 6 joints.
```bash
ros2 topic pub /joint_positions std_msgs/msg/Int32MultiArray "{data: [1000, 2000, 1500, 1000, 500, 0]}"
```
#### `estop` (`std_msgs/msg/Bool`)
Emergency stop override. If `true`, the arm will stop moving as fast as possible and hold position
```bash
ros2 topic pub /estop std_msgs/msg/Bool "{data: true}"
```

### Published Topics

#### `homing_done` (`std_msgs/msg/Bool`)
Publishes `true` once all joints and drills are homed.
```bash
ros2 topic echo /homing_done
```
### Services

#### `home_all` (`std_srvs/srv/Trigger`)
Initiates the homing sequence for all joints and drills in a configurable order with optional home offsets.
```bash
ros2 service call /home_all std_srvs/srv/Trigger
```
#### `set_motors_enabled` (`std_srvs/srv/SetBool`)
Enables or disables all motors via a service call.
```bash
ros2 service call /set_motors_enabled std_srvs/srv/SetBool "{data: true}"
```
#### `manual_home_wrist` (`std_srvs/srv/Trigger`)
Temporarily disables the wrist motors to allow manual positioning. After 10s it assumes they are homed and re-enables
```bash
ros2 service call /manual_home_wrist std_srvs/srv/Trigger
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
