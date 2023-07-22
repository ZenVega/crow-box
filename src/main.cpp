#include <Arduino.h>
#include <avr/sleep.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;
Stepper myStepper = Stepper(stepsPerRevolution, 4, 6, 5, 7);
const int coinPin = 2;
const int calibrationPin = 3;
int buttonState = 0;

#include "constants.h"

bool dispensing = false;
bool calibrating = false;

void dispense_peanuts()
{
  Serial.println("Dispensing through interrupt");
  detachInterrupt(digitalPinToInterrupt(coinPin));
  dispensing = true;
}

void init_calibration_mode()
{
  Serial.println("Calibration mode");
  detachInterrupt(digitalPinToInterrupt(calibrationPin));
  calibrating = true;
}

// void turnWheel()
// {
//   Serial.println("Turning wheel");
//   detachInterrupt(digitalPinToInterrupt(buttonPin));
//   myStepper.setSpeed(10);
//   myStepper.step(20);
// }

void setup()
{
  Serial.begin(9600);
  pinMode(coinPin, INPUT);
  digitalWrite(coinPin, HIGH); // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(coinPin), dispense_peanuts, FALLING);

  pinMode(calibrationPin, INPUT);
  digitalWrite(calibrationPin, HIGH); // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(calibrationPin), init_calibration_mode, FALLING);
}

void loop()
{
  if (dispensing)
  {
    Serial.println("Dispensing peanuts");
    myStepper.setSpeed(12);
    myStepper.step(-428);
    dispensing = false;
    attachInterrupt(digitalPinToInterrupt(coinPin), dispense_peanuts, FALLING);
  }
  else if (calibrating)
  {
    myStepper.setSpeed(12);
    myStepper.step(-10);
    delay(5);
    Serial.println("Calibrating");
    if (digitalRead(calibrationPin) == HIGH)
    {
      calibrating = false;
      attachInterrupt(digitalPinToInterrupt(calibrationPin), init_calibration_mode, FALLING);
    }
  }
}

// put function definitions here: