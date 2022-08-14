#ifndef SHARED_FRAME_H
#define SHARED_FRAME_H
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "vospi.h"

typedef struct {
  vospi_frame_t frame;
  SemaphoreHandle_t sem;
} sharedData;

#endif
