#pragma once
#include "configuration.h"
#include "motor_driver.h"

Motor joints[6] = {                                                                 
//Motor(STEP        , DIR        , CS        , ENABLE        , MICROSTEPS, ENDSTOP     , RSENSE    , CURRENT,   STEPS_PER_UNIT, SG HOMING, SGT    , HOMING DIR, HOMING SPEED, INVERT )
  Motor(X_STEP_PIN  , X_DIR_PIN  , X_CS_PIN  , X_ENABLE_PIN  , MICROSTEPS, X_STOP_PIN  , X_RSENSE  , J1_CURR  , J1_STEPS     , J1_SGH   , J1_SGT , J1_HDIR   , J1_HSPEED   , J1_INV  ), // Yaw
  Motor(Y_STEP_PIN  , Y_DIR_PIN  , Y_CS_PIN  , Y_ENABLE_PIN  , MICROSTEPS, Y_STOP_PIN  , Y_RSENSE  , J2_CURR  , J2_STEPS     , J2_SGH   , J2_SGT , J2_HDIR   , J2_HSPEED   , J2_INV  ), // Shoulder
  Motor(Z_STEP_PIN  , Z_DIR_PIN  , Z_CS_PIN  , Z_ENABLE_PIN  , MICROSTEPS, Z_STOP_PIN  , Z_RSENSE  , J3_CURR  , J3_STEPS     , J3_SGH   , J3_SGT , J3_HDIR   , J3_HSPEED   , J3_INV  ), // Elbow
  Motor(E0_STEP_PIN , E0_DIR_PIN , E0_CS_PIN , E0_ENABLE_PIN , MICROSTEPS, E0_DIAG_PIN , E0_RSENSE , J4_CURR  , J4_STEPS     , J4_SGH   , J4_SGT , J4_HDIR   , J4_HSPEED   , J4_INV  ), // Wrist
  Motor(E1_STEP_PIN , E1_DIR_PIN , E1_CS_PIN , E1_ENABLE_PIN , MICROSTEPS, E1_DIAG_PIN , E1_RSENSE , J5_CURR  , J5_STEPS     , J5_SGH   , J5_SGT , J5_HDIR   , J5_HSPEED   , J5_INV  ), // Wrist
  Motor(E2_STEP_PIN , E2_DIR_PIN , E2_CS_PIN , E2_ENABLE_PIN , MICROSTEPS, E3_DIAG_PIN , E2_RSENSE , J6_CURR  , J6_STEPS     , J6_SGH   , J6_SGT , J6_HDIR   , J6_HSPEED   , J6_INV  ) // Wrist
};

Motor drill1(E3_STEP_PIN, E3_DIR_PIN, E3_CS_PIN, E3_ENABLE_PIN, E3_DIAG_PIN, MICROSTEPS, E3_RSENSE, D1_CURR, D1_STEPS, false, 0, -1, 1, false);
Motor drill2(E4_STEP_PIN, E3_DIR_PIN, E4_CS_PIN, E4_ENABLE_PIN, E4_DIAG_PIN, MICROSTEPS, E4_RSENSE, D2_CURR, D2_STEPS, false, 0, -1, 1, false);
