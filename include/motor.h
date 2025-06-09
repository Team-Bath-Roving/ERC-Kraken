#pragma once
#include <AccelStepper.h>
#include <TMCStepper.h>
#include "kraken_pinout.h"

class Motor {
private:
  AccelStepper stepper;
  TMC2160Stepper driver;

  uint8_t cs_pin, en_pin;
  uint8_t endstop_pin;
  int steps_per_rev;
  int homing_dir;
  int homing_speed;
  int sgt;
  bool sg_homing;
  bool homed = false;

public:
  Motor(uint8_t step_pin, uint8_t dir_pin, uint8_t cs_pin, uint8_t en_pin,
         uint8_t endstop_pin, float RSense, int current_mA, int steps_per_rev,
         bool sg_homing, int sgt,
        int homing_dir, int homing_speed, bool invert_dir)
    : stepper(AccelStepper::DRIVER, step_pin, dir_pin),
      driver(cs_pin, RSense,TMC_SPI_MOSI,TMC_SPI_MISO,TMC_SPI_SCK,-1),
      cs_pin(cs_pin),
      en_pin(en_pin),
      endstop_pin(endstop_pin),
      homing_dir(homing_dir),
      homing_speed(abs(homing_speed)),
      steps_per_rev(steps_per_rev),
      sgt(sgt),
      sg_homing(sg_homing)
  {
    stepper.setPinsInverted(invert_dir, false, false);
  }

  void begin() {
    pinMode(en_pin, OUTPUT);
    digitalWrite(en_pin, LOW); // Enable driver

    driver.begin();
    driver.toff(5);
    driver.rms_current(driverCurrentClamp());
    driver.microsteps(16);
    // driver.en_spreadCycle(true);
    driver.pwm_autoscale(false);
    // driver.stealthChop(false);

    if (sg_homing) {
      driver.TCOOLTHRS(0xFFFF);
      driver.semin(5);
      driver.semax(2);
      driver.sedn(0b01);
      driver.sgt(50);
    }

    stepper.setMaxSpeed(1000);
  }

  void run() {
    stepper.run();
  }

  void moveTo(int32_t pos) {
    stepper.moveTo(pos);
  }

  bool isRunning() {
    return stepper.distanceToGo() != 0;
  }

  void setCurrentPosition(int32_t pos) {
    stepper.setCurrentPosition(pos);
  }

  int32_t currentPosition() {
    return stepper.currentPosition();
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

  int32_t getPosition() {
    return stepper.currentPosition();
  }


private:
  int driverCurrentClamp() {
    return constrain(driver.rms_current(), 200, 2000);
  }
};
