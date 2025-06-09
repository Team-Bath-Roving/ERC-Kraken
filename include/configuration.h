#pragma once
#include "kraken_pinout.h"

#define NUM_JOINTS 6

#define ESTOP_PIN PC_2 // FWS Filament Sensor Port

// the following are in steps/s
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

// Current in mA
#define J1_CURR 800
#define J2_CURR 800
#define J3_CURR 800
#define J4_CURR 800
#define J5_CURR 800
#define J6_CURR 800
#define D1_CURR 800
#define D2_CURR 800

// Steps per revolution (of the axis, default 200 with no gearing)
#define J1_STEPS 200
#define J2_STEPS 200
#define J3_STEPS 200
#define J4_STEPS 200
#define J5_STEPS 200
#define J6_STEPS 200
#define D1_STEPS 800
#define D2_STEPS 800

/* --------------------------------- HOMING --------------------------------- */

// Sensorless Homing
#define J1_SGH false
#define J2_SGH false
#define J3_SGH false
#define J4_SGH false
#define J5_SGH false
#define J6_SGH false
#define D1_SGH false
#define D2_SGH false

// Sensorless Homing Threshold (sgt)
#define J1_SGT 0
#define J2_SGT 0
#define J3_SGT 0
#define J4_SGT 0
#define J5_SGT 0
#define J6_SGT 0
#define D1_SGT 0
#define D2_SGT 0

// Homing Direction
#define J1_HDIR -1
#define J2_HDIR -1
#define J3_HDIR -1
#define J4_HDIR -1
#define J5_HDIR -1
#define J6_HDIR -1
#define D1_HDIR -1
#define D2_HDIR -1

// Homing Speed (units/s)
#define J1_HSPEED 1
#define J2_HSPEED 1
#define J3_HSPEED 1
#define J4_HSPEED 1
#define J5_HSPEED 1
#define J6_HSPEED 1
#define D1_HSPEED 1
#define D2_HSPEED 1

// 
