#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "include/vospi.h"

static const char* TAG = "VOSPIDriver";
static spi_device_handle_t spi;

#define CAMERA_SPI_HOST SPI3_HOST
#define CAMERA_SPI_SPEED_HZ 20000000
#define CAMERA_VIDEO_WIDTH 160
#define CAMERA_VIDEO_HEIGHT 120

/**
 * Perform a SPI read.
 */
int spi_read(void* buf, uint32_t len)
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.rxlength = len * 8;
  t.tx_buffer = NULL;
  t.rx_buffer = buf;
  ret = spi_device_transmit(spi, &t);
  assert(ret == ESP_OK);
  return ret;
}

/**
 * Initialise the VoSPI interface.
 */
int vospi_init(uint8_t GpioMosiPin, uint8_t GpioMisoPin, uint8_t GpioClkPin, uint8_t GpioCsPin)
{
  ESP_LOGI(TAG, "starting VoSPI initialisation...");

  esp_err_t ret;
  spi_bus_config_t buscfg = {
    .miso_io_num=GpioMisoPin,
    .mosi_io_num=GpioMosiPin,
    .sclk_io_num=GpioClkPin,
    .max_transfer_sz=50000,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1
  };

  spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .clock_speed_hz = CAMERA_SPI_SPEED_HZ,
    .mode = 3,
    .spics_io_num = GpioCsPin,
    .queue_size = 1,
    .flags = SPI_DEVICE_HALFDUPLEX,
    .cs_ena_pretrans = 10
  };

  ret=spi_bus_initialize(CAMERA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  assert(ret==ESP_OK);

  ret=spi_bus_add_device(CAMERA_SPI_HOST, &devcfg, &spi);
  assert(ret==ESP_OK);

  return ret;
}

/**
 * Transfer a single VoSPI segment.
 * Returns the number of successfully-transferred segments (0 or 1).
 */
int transfer_segment(vospi_segment_t* segment)
{
  // Perform the spidev transfer
  if (spi_read(&segment->packets[0], VOSPI_PACKET_BYTES) != ESP_OK) {
    ESP_LOGE(TAG, "failed to transfer packet");
    return 0;
  }

  // Flip the byte order of the ID & CRC
  segment->packets[0].id = FLIP_WORD_BYTES(segment->packets[0].id);
  segment->packets[0].crc = FLIP_WORD_BYTES(segment->packets[0].crc);

  while ((segment->packets[0].id & 0x0f00) == 0x0f00) {
    spi_read(&segment->packets[0], VOSPI_PACKET_BYTES);
  }

  // Read the remaining packets
  if (spi_read(&segment->packets[1], VOSPI_PACKET_BYTES * (segment->packet_count - 1)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to transfer the rest of the segment");
    return 0;
  }

  // Flip the byte order for the rest of the packet IDs
  for (int i = 1; i < segment->packet_count; i ++) {
    segment->packets[i].id = FLIP_WORD_BYTES(segment->packets[i].id);
    segment->packets[i].crc = FLIP_WORD_BYTES(segment->packets[i].crc);
  }

  return 1;
}

/**
 * Synchroise the VoSPI stream and transfer a single frame.
 * Returns the number of successfully-transferred frames (0 or 1).
 */
int sync_and_transfer_frame(vospi_frame_t* frame)
{
  // Keep streaming segments until we receive a valid, first segment to sync
  ESP_LOGD(TAG, "synchronising with first segment");
  uint16_t packet_20_num;
  uint8_t ttt_bits, resets = 0;

  while (1) {

      // Stream a first segment
      if (!transfer_segment(&frame->segments[0])) {
        ESP_LOGE(TAG, "failed to receive the first segment");
        return 0;
      }

      // If the packet number isn't even correct, we'll reset the bus to sync
      packet_20_num = frame->segments[0].packets[20].id & 0xff;
      if (packet_20_num != 20) {
        // Deselect the chip, wait 200ms with CS deasserted
        ESP_LOGW(
          TAG,
          "packet 20 ID was %d (%02x)- deasserting CS & waiting to reset...",
          packet_20_num,
          frame->segments[0].packets[20].id
        );
        vTaskDelay(200 / portTICK_PERIOD_MS);

        if (++resets >= VOSPI_MAX_SYNC_RESETS) {
          ESP_LOGE(TAG, "too many resets while synchronising (%d)", resets);
          return 0;
        }
        
        continue;
      }

      // Check we're looking at the first segment, if not, just keep reading until we get there
      ttt_bits = frame->segments[0].packets[20].id >> 12;
      ESP_LOGD(TAG, "TTT bits were: %d P20 Num: %d", ttt_bits, packet_20_num);
      if (ttt_bits == 1) {
        break;
      }
  }

  // Receive the remaining segments
  for (int seg = 1; seg < VOSPI_SEGMENTS_PER_FRAME; seg ++) {
    transfer_segment(&frame->segments[seg]);
  }

  return 1;
}

/**
 * Transfer a frame.
 * Assumes that we're already synchronised with the VoSPI stream.
 */
int transfer_frame(vospi_frame_t* frame)
{
  uint8_t ttt_bits, restarts = 0;

  // Receive all segments
  for (int seg = 0; seg < VOSPI_SEGMENTS_PER_FRAME; seg ++) {
    transfer_segment(&frame->segments[seg]);
    ttt_bits = frame->segments[seg].packets[20].id >> 12;
    if (ttt_bits != seg + 1) {
      seg --;
      if (restarts ++ > VOSPI_MAX_INVALID_FRAMES * 4) {
        ESP_LOGE(TAG, "too many invalid frames - need to resync");
        return 0;
      }
      continue;
    }
  }

  return 1;
}
