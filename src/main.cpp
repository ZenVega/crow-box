#include <Arduino.h>
#include <Stepper.h>
#include "constants.h"

// TODO: Camera activation

Stepper myStepper = Stepper(STEPS_PER_REVOLUTION, STEPPER_PIN_1, STEPPER_PIN_3, STEPPER_PIN_2, STEPPER_PIN_4);
int buttonState = 0;

bool dispensing = false;
bool calibrating = false;

void dispense_peanuts()
{
  Serial.println("Dispensing through interrupt");
  detachInterrupt(digitalPinToInterrupt(COIN_INTERRUPT_PIN));
  dispensing = true;
}
void IRAM_ATTR isrCoinInterrupt()
{
  dispense_peanuts();
}

void IRAM_ATTR isrCalibrationInterrupt()
{
  Serial.println("Calibration mode");
  detachInterrupt(digitalPinToInterrupt(CALIBRATION_INTERRUPT_PIN));
  calibrating = true;
}

void setup()
{
  Serial.begin(9600);
  pinMode(COIN_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COIN_INTERRUPT_PIN), isrCoinInterrupt, FALLING);

  pinMode(CALIBRATION_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIBRATION_INTERRUPT_PIN), isrCalibrationInterrupt, FALLING);
}

void loop()
{
  Serial.println("Monitor");
  if (dispensing)
  {
    Serial.println("Dispensing peanuts");
    myStepper.setSpeed(STEPPER_SPEED);
    myStepper.step(STEPS_PER_PEANUT);
    dispensing = false;
    attachInterrupt(digitalPinToInterrupt(COIN_INTERRUPT_PIN), dispense_peanuts, FALLING);
  }
  while (calibrating)
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
  }
  delay(1000);
}
