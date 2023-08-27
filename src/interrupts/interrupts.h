#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <Arduino.h>

void IRAM_ATTR isrCoinInterrupt();
void IRAM_ATTR isrPiezoInterrupt();
void IRAM_ATTR isrCalibrationInterrupt();

#endif // INTERRUPTS_H
