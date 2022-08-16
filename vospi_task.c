#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/vospi.h"
#include "include/shared_frame.h"

#define RestartTime 500

static const char *TAG = "VoSPITask";

static bool CameraInitialized = false;
static uint8_t GpioPowerPin = 0;

void SetupPowerReboot(uint8_t gpioPowerPin)
{
  gpio_pad_select_gpio(gpioPowerPin);
  gpio_set_direction(gpioPowerPin, GPIO_MODE_OUTPUT);
  GpioPowerPin = gpioPowerPin;
}

void RunPowerReboot()
{
  if (GpioPowerPin == 0)
  {
    ESP_LOGE(TAG, "Power pin not initialized.");
    return;
  }

  gpio_set_level(GpioPowerPin, 0);
  vTaskDelay(RestartTime / portTICK_PERIOD_MS);
  gpio_set_level(GpioPowerPin, 1);
  vTaskDelay(RestartTime / portTICK_PERIOD_MS);
}

void InitCamera(uint8_t gpioPowerPin, uint16_t restartTime, uint8_t GpioMosiPin, uint8_t GpioMisoPin, uint8_t GpioClkPin, uint8_t GpioCsPin)
{
  ESP_LOGI(TAG, "Starting camera initialisation...");
  SetupPowerReboot(gpioPowerPin);
  vospi_init(GpioMosiPin, GpioMisoPin, GpioClkPin, GpioCsPin);

  CameraInitialized = true;
}

void vospi_task(sharedData *c_frame)
{
  if (!CameraInitialized)
  {
    ESP_LOGE(TAG, "Camera not initialized.");
    return;
  }

  ESP_LOGI(TAG, "Start VoSPI task...");

  // Allocate space (32-bit multiple for DMA)
  ESP_LOGI(TAG, "preallocating space for segments... (%d bytes)", (sizeof(vospi_frame_t) / 4) * 4);
  vospi_frame_t *frame = heap_caps_malloc((sizeof(vospi_frame_t) / 4) * 4, MALLOC_CAP_DMA);
  for (int seg = 0; seg < VOSPI_SEGMENTS_PER_FRAME; seg++)
  {
    frame->segments[seg].packet_count = VOSPI_PACKETS_PER_SEGMENT_NORMAL;
  }

  while (1)
  {
    // Synchronise and transfer a single frame
    ESP_LOGI(TAG, "aquiring VoSPI synchronisation");
    if (0 == sync_and_transfer_frame(frame))
    {
      ESP_LOGE(TAG, "failed to obtain frame from device.");
      continue;
    }

    ESP_LOGI(TAG, "VoSPI stream synchronised");
    xSemaphoreGive(c_frame->sem);

    while (1)
    {
      if (!transfer_frame(frame))
      {
        ESP_LOGI(TAG, "resynchronising...");
        break;
      }

      // Obtain the semaphore to update the current frame, if we can't update
      // the current frame, just drop this one and try again next time
      if (xSemaphoreTake(c_frame->sem, 0) != pdTRUE)
      {
        ESP_LOGW(TAG, "couldn't obtain c_frame sem, dropping frame");
        continue;
      }

      // Copy the frame into place
      memcpy(&c_frame->frame, frame, sizeof(vospi_frame_t));
      xSemaphoreGive(c_frame->sem);
    }
  }
}
