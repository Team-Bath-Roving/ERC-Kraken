#pragma once
#include "kraken_pinout.h"


#define NUM_JOINTS 7

const char * joint_names[NUM_JOINTS] = {
  "shoulder_pan_joint", "shoulder_lift_joint",
  "elbow_joint", "wrist_1_joint",
  "wrist_2_joint", "wrist_3_joint",
  "joint_7"
};




#define ESTOP_PIN PC_2 // FWS Filament Sensor Port

// the following are in steps
#define DEFAULT_ACCEL 1000
#define DEFAULT_SPEED 500

/* ----------------------------- Motor Settings ----------------------------- */

#define MICROSTEPS 32 // gets interpolated to 256

// Motor Direction Invert
#define J1_INV false
#define J2_INV false
#define J3_INV false
#define J4_INV false
#define J5_INV false
#define J6_INV false
#define J7_INV false
#define D1_INV false

// Current in mA
#define J1_CURR 800
#define J2_CURR 800
#define J3_CURR 800
#define J4_CURR 800
#define J5_CURR 800
#define J6_CURR 800
#define J7_CURR 800
#define D1_CURR 800

// Steps per unit (of the axis, default 200 with no gearing)
#define J1_STEPS 200 // Direct drive
#define J2_STEPS 200 * 20/1 // Cycloidal drive
#define J3_STEPS 200 * 20/1 // Cycloidal drive
#define J4_STEPS 200 // Direct drive
#define J5_STEPS 200 * 24/12 // Semi differential pulleys
#define J6_STEPS 200 * 24/12 // Semi differential pulleys
#define J7_STEPS 200 // Direct drive or as needed
#define D1_STEPS 200

/* --------------------------------- HOMING --------------------------------- */

// Sensorless Homing
#define J1_SGH false
#define J2_SGH false
#define J3_SGH false
#define J4_SGH false
#define J5_SGH false
#define J6_SGH false
#define J7_SGH false
#define D1_SGH false

// Sensorless Homing Threshold (sgt)
#define J1_SGT 0
#define J2_SGT 0
#define J3_SGT 0
#define J4_SGT 0
#define J5_SGT 0
#define J6_SGT 0
#define J7_SGT 0
#define D1_SGT 0

// Homing Direction
#define J1_HDIR -1
#define J2_HDIR -1
#define J3_HDIR -1
#define J4_HDIR -1
#define J5_HDIR -1
#define J6_HDIR -1
#define J7_HDIR -1
#define D1_HDIR -1

// Homing Speed (units/s)
#define J1_HSPEED 1
#define J2_HSPEED 1
#define J3_HSPEED 1
#define J4_HSPEED 1
#define J5_HSPEED 1
#define J6_HSPEED 1
#define J7_HSPEED 1
#define D1_HSPEED 1

// Home Position (units)
#define J1_HPOS 0
#define J2_HPOS 0
#define J3_HPOS 0
#define J4_HPOS 0
#define J5_HPOS 0
#define J6_HPOS 0
#define J7_HPOS 0
#define D1_HPOS 0
