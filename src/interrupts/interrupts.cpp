#include "interrupts.h"
#include "../constants.h"

extern bool dispensing;
extern bool buzzing;
extern bool calibrating;

void IRAM_ATTR isrCoinInterrupt()
{
  Serial.println("Dispensing through interrupt");
  detachInterrupt(digitalPinToInterrupt(COIN_INTERRUPT_PIN));
  dispensing = true;
}

void IRAM_ATTR isrPiezoInterrupt()
{
  Serial.println("Wakey wakey");
  detachInterrupt(digitalPinToInterrupt(PIEZO_INTERRUPT_PIN));
  buzzing = true;
}

void IRAM_ATTR isrCalibrationInterrupt()
{
  Serial.println("Calibration mode");
  detachInterrupt(digitalPinToInterrupt(CALIBRATION_INTERRUPT_PIN));
  calibrating = true;
}
