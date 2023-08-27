#include <Arduino.h>
#include <Stepper.h>
#include "constants.h"
#include "interrupts/interrupts.h"

// TODO: Camera activation

Stepper myStepper = Stepper(STEPS_PER_REVOLUTION, STEPPER_PIN_1, STEPPER_PIN_3, STEPPER_PIN_2, STEPPER_PIN_4);

bool dispensing = false;
bool calibrating = false;
bool buzzing = false;

void dispense_peanuts()
{
  Serial.println("Dispensing peanuts");
  myStepper.setSpeed(STEPPER_SPEED);
  myStepper.step(STEPS_PER_PEANUT);
  dispensing = false;
  attachInterrupt(digitalPinToInterrupt(COIN_INTERRUPT_PIN), isrCoinInterrupt, FALLING);
};

void piezo_action()
{
  Serial.println("Buzzing");
  buzzing = false;
  attachInterrupt(digitalPinToInterrupt(PIEZO_INTERRUPT_PIN), isrPiezoInterrupt, FALLING);
};

void calibrate()
{
  myStepper.setSpeed(STEPPER_SPEED);
  myStepper.step(STEPS_PER_CALIBRATION);
  delay(5);
  Serial.println("Calibrating");
  if (digitalRead(CALIBRATION_INTERRUPT_PIN) == HIGH)
  {
    calibrating = false;
    attachInterrupt(digitalPinToInterrupt(CALIBRATION_INTERRUPT_PIN), isrCalibrationInterrupt, FALLING);
    // TODO: Sleepmode
  }
};

void setup()
{
  Serial.begin(9600);
  pinMode(COIN_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COIN_INTERRUPT_PIN), isrCoinInterrupt, FALLING);

  pinMode(CALIBRATION_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIBRATION_INTERRUPT_PIN), isrCalibrationInterrupt, FALLING);

  pinMode(PIEZO_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIEZO_INTERRUPT_PIN), isrPiezoInterrupt, FALLING);
};

void loop()
{
  if (buzzing)
  {
    piezo_action();
  }
  if (dispensing)
  {
    dispense_peanuts();
  }

  while (calibrating)
  {
    calibrate();
  }
  delay(100);
};
