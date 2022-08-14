#ifndef VOSPI_TASK_H
#define VOSPI_TASK_H

#include "esp_system.h"
#include "shared_frame.h"

void RunPowerReboot();
void InitCamera(uint8_t gpioPowerPin, uint16_t restartTime, uint8_t GpioMosiPin, uint8_t GpioMisoPin, uint8_t GpioClkPin, uint8_t GpioCsPin);
void vospi_task(sharedData *c_frame);

#endif
