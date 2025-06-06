/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

// #include "env_validate.h"

#define BOARD_INFO_NAME   "BTT Kraken V1.0"
#define BOARD_WEBSITE_URL "github.com/bigtreetech/BIGTREETECH-Kraken"

#define USES_DIAG_JUMPERS

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 8

//
// Servos
//
#define SERVO0_PIN                          PE9   // PROBE
#define SERVO1_PIN                          PE7   // MOTOR

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PC15  // MIN1
#define Y_DIAG_PIN                          PF0   // MIN2
#define Z_DIAG_PIN                          PF1   // MIN3
#define E0_DIAG_PIN                         PF2   // MIN4
#define E1_DIAG_PIN                         PF3   // MIN5
#define E2_DIAG_PIN                         PF4   // MIN6
#define E3_DIAG_PIN                         PF10  // MIN7
#define E4_DIAG_PIN                         PC0   // MIN8

//
// Limit Switches
//
#define X_STOP_PIN                    X_DIAG_PIN  // MIN1
#define Y_STOP_PIN                    Y_DIAG_PIN  // MIN2
#define Z_STOP_PIN                    Z_DIAG_PIN  // MIN3
#define X_OTHR_PIN                   E0_DIAG_PIN  // MIN4
#define Y_OTHR_PIN                   E1_DIAG_PIN  // MIN5
#define Z_OTHR_PIN                   E2_DIAG_PIN  // MIN6

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PG1   // PROBE (BLTouch, MicroProbe, etc.)
  //#define Z_MIN_PROBE_PIN                 PD11  // IND-DET (Inductive probe)
#endif


//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN               E0_DIAG_PIN  // MIN4
#define FIL_RUNOUT2_PIN              E1_DIAG_PIN  // MIN5
#define FIL_RUNOUT3_PIN              E2_DIAG_PIN  // MIN6
#define FIL_RUNOUT4_PIN              E3_DIAG_PIN  // MIN7
#define FIL_RUNOUT5_PIN              E4_DIAG_PIN  // MIN8

//
// Steppers
//
#define X_STEP_PIN                          PC14  // S1 (Motor 1)
#define X_DIR_PIN                           PC13
#define X_ENABLE_PIN                        PE6
#ifndef X_CS_PIN
  #define X_CS_PIN                          PD6
#endif

#define Y_STEP_PIN                          PE5   // S2 (Motor 2)
#define Y_DIR_PIN                           PE4
#define Y_ENABLE_PIN                        PE3
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD5
#endif

#define Z_STEP_PIN                          PE2   // S3 (Motor 3)
#define Z_DIR_PIN                           PE1
#define Z_ENABLE_PIN                        PE0
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD4
#endif

#define E0_STEP_PIN                         PB9   // S4 (Motor 4)
#define E0_DIR_PIN                          PB8
#define E0_ENABLE_PIN                       PB7
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD3
#endif

#define E1_STEP_PIN                         PG9   // S5 (Motor 5)
#define E1_DIR_PIN                          PG10
#define E1_ENABLE_PIN                       PG13
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD2
#endif

#define E2_STEP_PIN                         PG11  // S6 (Motor 6)
#define E2_DIR_PIN                          PD7
#define E2_ENABLE_PIN                       PG12
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PA15
#endif

#define E3_STEP_PIN                         PB4   // S7 (Motor 7)
#define E3_DIR_PIN                          PB3
#define E3_ENABLE_PIN                       PB5
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PA9
#endif

#define E4_STEP_PIN                         PG15  // S8 (Motor 8)
#define E4_DIR_PIN                          PB6
#define E4_ENABLE_PIN                       PG14
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PA10
#endif

//
// Integrated TMC2160 driver defaults
//

#define X_RSENSE 0.022
#define Y_RSENSE 0.022
#define Z_RSENSE 0.022
#define E0_RSENSE 0.022
#define E1_RSENSE 0.075
#define E2_RSENSE 0.075
#define E3_RSENSE 0.075
#define E4_RSENSE 0.075


//
// Temperature Sensors
//
#define TEMP_0_PIN                          PB1   // TH0
#define TEMP_1_PIN                          PC5   // TH1
#define TEMP_2_PIN                          PC4   // TH2
#define TEMP_3_PIN                          PA7   // TH3
#define TEMP_BED_PIN                        PB0   // THB

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PF5   // BED-OUT
#define HEATER_0_PIN                        PF6   // HE0
#define HEATER_1_PIN                        PF7   // HE1
#define HEATER_2_PIN                        PF9   // HE2
#define HEATER_3_PIN                        PF8   // HE3

#define FAN0_PIN                            PA0   // FAN0 (3 wire)
#define FAN1_PIN                            PA1   // FAN1 (3 wire)
#define FAN2_PIN                            PA2   // FAN2 (3 wire)
#define FAN3_PIN                            PA3   // FAN3 (3 wire)
#define FAN4_PIN                            PA4   // FAN4 (3 wire)
#define FAN5_PIN                            PA5   // FAN5 (3 wire)
#define FAN6_PIN                            PA6   // FAN6 (4 wire)
#define FAN7_PIN                            PE8   // FAN7 (4 wire)



//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PD10  // PS-ON
#endif

//
// Misc. Functions
//
#define LED_PIN                             PA14

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                      PC2
#endif
#ifndef FILWIDTH2_PIN
  #define FILWIDTH2_PIN                     PC3
#endif


//
// Default pins for TMC software SPI
// This board only supports SW SPI for stepper drivers
//
#define HAS_TMC_SPI true
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
  #ifndef TMC_SPI_MOSI
    #define TMC_SPI_MOSI                    PC8
  #endif
  #ifndef TMC_SPI_MISO
    #define TMC_SPI_MISO                    PC7
  #endif
  #ifndef TMC_SPI_SCK
    #define TMC_SPI_SCK                     PC6
  #endif
#endif

/**               ------                                      ------
 * (BEEPER) PG5  | 1  2 | PG4  (BTN_ENC)         (MISO) PE13 | 1  2 | PE12 (SCK)
 * (LCD_EN) PG3  | 3  4 | PG2  (LCD_RS)       (BTN_EN1) PG8  | 3  4 | PE11 (SD_SS)
 * (LCD_D4) PD15   5  6 | PD14 (LCD_D5)       (BTN_EN2) PG7    5  6 | PE14 (MOSI)
 * (LCD_D6) PD13 | 7  8 | PD12 (LCD_D7)     (SD_DETECT) PG6  | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | --
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PG5
#define EXP1_02_PIN                         PG4
#define EXP1_03_PIN                         PG3
#define EXP1_04_PIN                         PG2
#define EXP1_05_PIN                         PD15
#define EXP1_06_PIN                         PD14
#define EXP1_07_PIN                         PD13
#define EXP1_08_PIN                         PD12

#define EXP2_01_PIN                         PE13
#define EXP2_02_PIN                         PE12
#define EXP2_03_PIN                         PG8
#define EXP2_04_PIN                         PE11
#define EXP2_05_PIN                         PG7
#define EXP2_06_PIN                         PE14
#define EXP2_07_PIN                         PG6
#define EXP2_08_PIN                         -1


//
// NeoPixel LED
//
// #if DISABLED(FYSETC_MINI_12864_2_1) && !defined(NEOPIXEL_PIN)
#define NEOPIXEL_PIN                      PF12  // RGB1

#ifndef NEOPIXEL2_PIN
  #define NEOPIXEL2_PIN                     PF11  // RGB2
#endif