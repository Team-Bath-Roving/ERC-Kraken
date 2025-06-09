#pragma once
#include <AccelStepper.h>
#include <TMCStepper.h>
#include "kraken_pinout.h"
#include "configuration.h"

class Motor {
private:
  AccelStepper stepper;
  TMC2160Stepper driver;

  uint8_t cs_pin, en_pin;
  uint8_t endstop_pin;
  int steps_per_unit;
  int homing_dir;
  int homing_speed;
  int sgt;
  int acceleration=DEFAULT_ACCEL;
  int speed=DEFAULT_SPEED;
  uint16_t microsteps;
  bool sg_homing;
  bool homed = false;

public:
  Motor(uint8_t step_pin, uint8_t dir_pin, uint8_t cs_pin, uint8_t en_pin, uint microsteps,
        uint8_t endstop_pin, float RSense, int current_mA, int steps_per_unit,
        bool sg_homing, int sgt,
        int homing_dir, int homing_speed, bool invert_dir)
    : stepper(AccelStepper::DRIVER, step_pin, dir_pin),
      driver(cs_pin, RSense,TMC_SPI_MOSI,TMC_SPI_MISO,TMC_SPI_SCK,-1),
      cs_pin(cs_pin),
      en_pin(en_pin),
      endstop_pin(endstop_pin),
      homing_dir(homing_dir),
      homing_speed(abs(homing_speed)),
      steps_per_unit(steps_per_unit),
      sgt(sgt),
      sg_homing(sg_homing),
      microsteps(microsteps)
  {
    stepper.setPinsInverted(invert_dir, false, false);
  }

  void begin() {
    pinMode(en_pin, OUTPUT);
    digitalWrite(en_pin, LOW); // Enable driver

    driver.begin();
    driver.toff(5);
    driver.rms_current(driverCurrentClamp());
    driver.microsteps(microsteps);
    // driver.en_spreadCycle(true);
    driver.pwm_autoscale(false); // disable stealthchop
    // driver.stealthChop(false);

    if (sg_homing) {
      driver.TCOOLTHRS(0xFFFF);
      driver.semin(5);
      driver.semax(2);
      driver.sedn(0b01);
      driver.sgt(50);
    }
    stepper.setAcceleration(DEFAULT_ACCEL);

    stepper.setMaxSpeed(DEFAULT_SPEED);
    enable();
  }
  void enable() {
    stepper.enableOutputs();
  }
  void disable() {
    stepper.disableOutputs();
  }
  void run() {
    stepper.run();
    
  }
  void stop() {
    driver.stop_enable(true); // set the estop register to halt as quick as possible
    stepper.stop(); // set target position to stop as well
  }

  void moveTo(float pos) {
    stepper.moveTo((long)(pos*steps_per_unit));
  }

  bool isRunning() {
    return stepper.distanceToGo() != 0;
  }

  void setCurrentPosition(float pos) {
    stepper.setCurrentPosition((long)(pos*steps_per_unit));
  }

  float currentPosition() {
    return (float)stepper.currentPosition()/steps_per_unit;
  }

  bool isHomed() const {
    return homed;
  }

  void home() {
    stepper.setMaxSpeed(homing_speed);

    while (digitalRead(endstop_pin) == HIGH) {
      stepper.moveTo(stepper.currentPosition() + homing_dir * homing_speed);
      stepper.run();
    }

    stepper.setCurrentPosition(0);
    stepper.moveTo(-homing_dir * 100);
    while (stepper.distanceToGo() != 0) stepper.run();

    homed = true;
  }

  AccelStepper& getStepper() {
    return stepper;
  }

private:
  int driverCurrentClamp() {
    return constrain(driver.rms_current(), 200, 2000);
  }
};
