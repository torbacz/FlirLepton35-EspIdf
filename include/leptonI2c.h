#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_log.h"

#define LEP_I2C_DEVICE_ADDRESS (uint8_t)0x2A

#define LEP_I2C_COMMAND_MODULE_ID_BIT_MASK      (uint16_t)0x0F00
#define LEP_I2C_COMMAND_ID_BIT_MASK             (uint16_t)0x00FC
#define LEP_I2C_COMMAND_TYPE_BIT_MASK           (uint16_t)0x0003

#define LEP_I2C_COMMAND_TYPE_GET                (uint16_t)0x0000
#define LEP_I2C_COMMAND_TYPE_SET                (uint16_t)0x0001
#define LEP_I2C_COMMAND_TYPE_RUN                (uint16_t)0x0002

#define LEP_I2C_STATUS_BUSY_BIT_MASK            (uint16_t)0x0001
#define LEP_I2C_STATUS_BOOT_MODE_BIT_MASK       (uint16_t)0x0002
#define LEP_I2C_STATUS_BOOT_STATUS_BIT_MASK     (uint16_t)0x0004
#define LEP_I2C_STATUS_ERROR_CODE_BIT_MASK      (uint16_t)0xFF00
#define LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT     8

#define LEP_I2C_REG_BASE_ADDR                   (uint16_t)0x0000
#define LEP_I2C_POWER_REG                       (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0000)
#define LEP_I2C_STATUS_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0002)
#define LEP_I2C_COMMAND_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0004)
#define LEP_I2C_DATA_LENGTH_REG                 (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0006)
#define LEP_I2C_DATA_0_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0008)
#define LEP_I2C_DATA_1_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x000A)
#define LEP_I2C_DATA_2_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x000C)
#define LEP_I2C_DATA_3_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x000E)
#define LEP_I2C_DATA_4_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0010)
#define LEP_I2C_DATA_5_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0012)
#define LEP_I2C_DATA_6_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0014)
#define LEP_I2C_DATA_7_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0016)
#define LEP_I2C_DATA_8_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0018)
#define LEP_I2C_DATA_9_REG                      (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x001A)
#define LEP_I2C_DATA_10_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x001C)
#define LEP_I2C_DATA_11_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x001E)
#define LEP_I2C_DATA_12_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0020)
#define LEP_I2C_DATA_13_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0022)
#define LEP_I2C_DATA_14_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0024)
#define LEP_I2C_DATA_15_REG                     (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0026)
#define LEP_I2C_DATA_CRC_REG                    (uint16_t)(LEP_I2C_REG_BASE_ADDR + 0x0028)

#define LEP_I2C_DATA_BUFFER                     (uint16_t)0xF800
#define LEP_I2C_DATA_BUFFER_LENGTH              (uint16_t)0x0800

#define LEP_AGC_MODULE_BASE                     (uint16_t)0x0100
#define LEP_CID_AGC_ENABLE_STATE                (uint16_t)(LEP_AGC_MODULE_BASE + 0x0000)
#define LEP_CID_AGC_POLICY                      (uint16_t)(LEP_AGC_MODULE_BASE + 0x0004)
#define LEP_CID_AGC_ROI                         (uint16_t)(LEP_AGC_MODULE_BASE + 0x0008)
#define LEP_CID_AGC_STATISTICS                  (uint16_t)(LEP_AGC_MODULE_BASE + 0x000C)
#define LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT      (uint16_t)(LEP_AGC_MODULE_BASE + 0x0010)
#define LEP_CID_AGC_HISTOGRAM_TAIL_SIZE         (uint16_t)(LEP_AGC_MODULE_BASE + 0x0014)
#define LEP_CID_AGC_LINEAR_MAX_GAIN             (uint16_t)(LEP_AGC_MODULE_BASE + 0x0018)
#define LEP_CID_AGC_LINEAR_MIDPOINT             (uint16_t)(LEP_AGC_MODULE_BASE + 0x001C)
#define LEP_CID_AGC_LINEAR_DAMPENING_FACTOR     (uint16_t)(LEP_AGC_MODULE_BASE + 0x0020)
#define LEP_CID_AGC_HEQ_DAMPENING_FACTOR        (uint16_t)(LEP_AGC_MODULE_BASE + 0x0024)
#define LEP_CID_AGC_HEQ_MAX_GAIN                (uint16_t)(LEP_AGC_MODULE_BASE + 0x0028)
#define LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH         (uint16_t)(LEP_AGC_MODULE_BASE + 0x002C)
#define LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW          (uint16_t)(LEP_AGC_MODULE_BASE + 0x0030)
#define LEP_CID_AGC_HEQ_BIN_EXTENSION           (uint16_t)(LEP_AGC_MODULE_BASE + 0x0034)
#define LEP_CID_AGC_HEQ_MIDPOINT                (uint16_t)(LEP_AGC_MODULE_BASE + 0x0038)
#define LEP_CID_AGC_HEQ_EMPTY_COUNTS            (uint16_t)(LEP_AGC_MODULE_BASE + 0x003C)
#define LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR    (uint16_t)(LEP_AGC_MODULE_BASE + 0x0040)
#define LEP_CID_AGC_HEQ_SCALE_FACTOR            (uint16_t)(LEP_AGC_MODULE_BASE + 0x0044)
#define LEP_CID_AGC_CALC_ENABLE_STATE           (uint16_t)(LEP_AGC_MODULE_BASE + 0x0048)

#define LEP_SYS_MODULE_BASE                     (uint16_t)0x0200
#define LEP_CID_SYS_PING                        (uint16_t)(LEP_SYS_MODULE_BASE + 0x0000)
#define LEP_CID_SYS_CAM_STATUS                  (uint16_t)(LEP_SYS_MODULE_BASE + 0x0004)
#define LEP_CID_SYS_FLIR_SERIAL_NUMBER          (uint16_t)(LEP_SYS_MODULE_BASE + 0x0008)
#define LEP_CID_SYS_CAM_UPTIME                  (uint16_t)(LEP_SYS_MODULE_BASE + 0x000C)
#define LEP_CID_SYS_AUX_TEMPERATURE_KELVIN      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0010)
#define LEP_CID_SYS_FPA_TEMPERATURE_KELVIN      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0014)
#define LEP_CID_SYS_TELEMETRY_ENABLE_STATE      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0018)
#define LEP_CID_SYS_TELEMETRY_LOCATION          (uint16_t)(LEP_SYS_MODULE_BASE + 0x001C)
#define LEP_CID_SYS_EXECTUE_FRAME_AVERAGE       (uint16_t)(LEP_SYS_MODULE_BASE + 0x0020)
#define LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE       (uint16_t)(LEP_SYS_MODULE_BASE + 0x0024)
#define LEP_CID_SYS_CUST_SERIAL_NUMBER          (uint16_t)(LEP_SYS_MODULE_BASE + 0x0028)
#define LEP_CID_SYS_SCENE_STATISTICS            (uint16_t)(LEP_SYS_MODULE_BASE + 0x002C)
#define LEP_CID_SYS_SCENE_ROI                   (uint16_t)(LEP_SYS_MODULE_BASE + 0x0030)
#define LEP_CID_SYS_THERMAL_SHUTDOWN_COUNT      (uint16_t)(LEP_SYS_MODULE_BASE + 0x0034)
#define LEP_CID_SYS_SHUTTER_POSITION            (uint16_t)(LEP_SYS_MODULE_BASE + 0x0038)
#define LEP_CID_SYS_FFC_SHUTTER_MODE            (uint16_t)(LEP_SYS_MODULE_BASE + 0x003C)
#define LEP_CID_SYS_RUN_FFC                     (uint16_t)(LEP_SYS_MODULE_BASE + 0x0042)
#define LEP_CID_SYS_FFC_STATUS                  (uint16_t)(LEP_SYS_MODULE_BASE + 0x0044)

#define LEP_VID_MODULE_BASE                     (uint16_t)0x0300
#define LEP_CID_VID_POLARITY_SELECT             (uint16_t)(LEP_VID_MODULE_BASE + 0x0000)
#define LEP_CID_VID_LUT_SELECT                  (uint16_t)(LEP_VID_MODULE_BASE + 0x0004)
#define LEP_CID_VID_LUT_TRANSFER                (uint16_t)(LEP_VID_MODULE_BASE + 0x0008)
#define LEP_CID_VID_FOCUS_CALC_ENABLE           (uint16_t)(LEP_VID_MODULE_BASE + 0x000C)
#define LEP_CID_VID_FOCUS_ROI                   (uint16_t)(LEP_VID_MODULE_BASE + 0x0010)
#define LEP_CID_VID_FOCUS_THRESHOLD             (uint16_t)(LEP_VID_MODULE_BASE + 0x0014)
#define LEP_CID_VID_FOCUS_METRIC                (uint16_t)(LEP_VID_MODULE_BASE + 0x0018)
#define LEP_CID_VID_SBNUC_ENABLE                (uint16_t)(LEP_VID_MODULE_BASE + 0x001C)
#define LEP_CID_VID_GAMMA_SELECT                (uint16_t)(LEP_VID_MODULE_BASE + 0x0020)
#define LEP_CID_VID_FREEZE_ENABLE               (uint16_t)(LEP_VID_MODULE_BASE + 0x0024)


#define LEP_I2C_RUN_REBOOT                      0x4842

typedef enum {
    LEP_SYS_FFC_SHUTTER_MODE_MANUAL = 0,
    LEP_SYS_FFC_SHUTTER_MODE_AUTO,
    LEP_SYS_FFC_SHUTTER_MODE_EXTERNAL
} LEP_SYS_FFC_SHUTTER_MODE_STATE;

void i2cInit(int16_t GpioScl, int16_t GpioSda);

void RunFFC();

LEP_SYS_FFC_SHUTTER_MODE_STATE GetShutterMode();
void SetShutterMode(LEP_SYS_FFC_SHUTTER_MODE_STATE state);

uint32_t GetUptime();

void RunOemReboot();