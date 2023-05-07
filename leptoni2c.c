#include "include/leptonI2c.h"

// Slave buffer size (not used)
#define I2C_MASTER_TX_BUF_LEN 0
#define I2C_MASTER_RX_BUF_LEN 0

#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1 

// I2C
#define I2C_MASTER_NUM     1
#define I2C_MASTER_FREQ_HZ 100000

static const char* TAG = "cci";

static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t cci_mutex;

void i2c_lock()
{
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
}

void i2c_unlock()
{
	xSemaphoreGive(i2c_mutex);
}

/**
 * Read esp-i2c-slave
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2c_master_read_slave(uint8_t addr7, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MODE_MASTER, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * Write esp-i2c-slave
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t i2c_master_write_slave(uint8_t addr7, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MODE_MASTER, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * Wait for busy to be clear in the status register
 *   Returns the 16-bit STATUS
 *   Returns 0x00010000 if there is a communication failure
 */
static uint32_t cci_wait_busy_clear()
{
	bool err = false;
	uint8_t buf[2] = {0x00, 0x07};

	// Wait for booted, not busy
	while (((buf[1] & 0x07) != 0x06) && !err) {
		// Write STATUS register address
		buf[0] = 0x00;
		buf[1] = 0x02;
		
		i2c_lock();
		if (i2c_master_write_slave(LEP_I2C_DEVICE_ADDRESS, buf, sizeof(buf)) != ESP_OK) {
			ESP_LOGE(TAG, "failed to set STATUS register");
			err = true;
		};

		// Read register - low bits in buf[1]
		if (i2c_master_read_slave(LEP_I2C_DEVICE_ADDRESS, buf, sizeof(buf)) != ESP_OK) {
			ESP_LOGE(TAG, "failed to read STATUS register");
			err = true;
		}
		i2c_unlock();
	}
	
	if (err) {
		return 0x00010000;
	} else {
		return (buf[0] << 8) | buf[1];
	}
}

static int cci_write_register(uint16_t reg, uint16_t value)
{
	// Write the register address and value
	uint8_t write_buf[4] = {
		reg >> 8 & 0xff,
		reg & 0xff,
		value >> 8 & 0xff,
		value & 0xff
	};

	i2c_lock();
	if (i2c_master_write_slave(LEP_I2C_DEVICE_ADDRESS, write_buf, sizeof(write_buf)) != ESP_OK) {
		i2c_unlock();
		ESP_LOGE(TAG, "failed to write CCI register %02x with value %02x", reg, value);
		return -1;
	};
	i2c_unlock();

	return 1;
}

/**
 * Read a CCI register.
 */
static uint16_t cci_read_register(uint16_t reg)
{
	uint8_t buf[2];

	// Write the register address
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
  
	i2c_lock();
	if (i2c_master_write_slave(LEP_I2C_DEVICE_ADDRESS, buf, sizeof(buf)) != ESP_OK) {
		i2c_unlock();
		ESP_LOGE(TAG, "failed to write CCI register %02x", reg);
		return -1;
	}

	// Read
	if (i2c_master_read_slave(LEP_I2C_DEVICE_ADDRESS, buf, sizeof(buf)) != ESP_OK) {
		ESP_LOGE(TAG, "failed to read from CCI register %02x", reg);
	}
	i2c_unlock();

	return buf[0] << 8 | buf[1];
}

static void cci_wait_busy_clear_check(char* cmd)
{
	int8_t   response;
	uint32_t t32;
	
	//cci_last_status_error = false;
	
	t32 = cci_wait_busy_clear();
	//cci_last_status = t32 & 0xFFFF;
	if (t32 == 0x00010000) {
		ESP_LOGE(TAG, "cmd: %s failed wait_busy_clear", cmd);
		//cci_last_status_error = true;
	} else {
		response = (int8_t) ((t32 & 0x0000FF00) >> 8);
		if (response < 0) {
			ESP_LOGE(TAG, "%s returned %d", cmd, response);
		}
	}
}

uint16_t cmdCode(uint16_t cmdID, uint16_t cmdType) {
    return (cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) | (cmdID & LEP_I2C_COMMAND_ID_BIT_MASK) | (cmdType & LEP_I2C_COMMAND_TYPE_BIT_MASK);
}

void RunFFC()
{
	xSemaphoreTake(cci_mutex, portMAX_DELAY);
	cci_wait_busy_clear();
	cci_write_register(LEP_I2C_COMMAND_REG, cmdCode(LEP_CID_SYS_RUN_FFC, LEP_I2C_COMMAND_TYPE_RUN));
	cci_wait_busy_clear_check("RunFFC");
	xSemaphoreGive(cci_mutex);
}

LEP_SYS_FFC_SHUTTER_MODE_STATE GetShutterMode()
{
    xSemaphoreTake(cci_mutex, portMAX_DELAY);
	cci_wait_busy_clear();
	cci_write_register(LEP_I2C_DATA_LENGTH_REG, 2);
	cci_write_register(LEP_I2C_COMMAND_REG, LEP_CID_SYS_FFC_SHUTTER_MODE);
	cci_wait_busy_clear_check("GetShutterMode");
	xSemaphoreGive(cci_mutex);
	
	uint16_t ls_word = cci_read_register(LEP_I2C_DATA_0_REG);
	uint16_t ms_word = cci_read_register(LEP_I2C_DATA_1_REG);
	return (LEP_SYS_FFC_SHUTTER_MODE_STATE)(ms_word << 16 | ls_word);
}


void SetShutterMode(LEP_SYS_FFC_SHUTTER_MODE_STATE state)
{
	uint32_t value = state;
	
	xSemaphoreTake(cci_mutex, portMAX_DELAY);
	cci_wait_busy_clear();
	cci_write_register(LEP_I2C_DATA_0_REG, value & 0xffff);
	cci_write_register(LEP_I2C_DATA_1_REG, value >> 16 & 0xffff);
	cci_write_register(LEP_I2C_DATA_LENGTH_REG, 2);
	cci_write_register(LEP_I2C_COMMAND_REG, cmdCode(LEP_CID_SYS_FFC_SHUTTER_MODE, LEP_I2C_COMMAND_TYPE_SET));
	cci_wait_busy_clear_check("SetShutterMode");
	xSemaphoreGive(cci_mutex);
}

uint32_t GetUptime()
{
	xSemaphoreTake(cci_mutex, portMAX_DELAY);
	cci_wait_busy_clear();
	cci_write_register(LEP_I2C_DATA_LENGTH_REG, 2);
	cci_write_register(LEP_I2C_COMMAND_REG, LEP_CID_SYS_CAM_UPTIME);
	cci_wait_busy_clear_check("GetUptime");
	xSemaphoreGive(cci_mutex);
	
	uint16_t ls_word = cci_read_register(LEP_I2C_DATA_0_REG);
	uint16_t ms_word = cci_read_register(LEP_I2C_DATA_1_REG);
	return ms_word << 16 | ls_word;
}


esp_err_t i2c_master_init(int16_t GpioScl, int16_t GpioSda)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    
    i2c_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(i2c_mutex);
    
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GpioSda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GpioScl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    i2c_param_config(i2c_master_port, &conf);
    
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_LEN,
                              I2C_MASTER_TX_BUF_LEN, 0);
}

void i2cInit(int16_t GpioScl, int16_t GpioSda)
{
    cci_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(cci_mutex);
    ESP_ERROR_CHECK(i2c_master_init(GpioScl, GpioSda));
}

void RunOemReboot()
{
	xSemaphoreTake(cci_mutex, portMAX_DELAY);
	cci_wait_busy_clear();
	cci_write_register(LEP_I2C_COMMAND_REG, LEP_I2C_RUN_REBOOT);
	// Sleep to allow camera to reboot and run FFC
	vTaskDelay(pdMS_TO_TICKS(6000));
	cci_wait_busy_clear_check("RunOemReboot");
	xSemaphoreGive(cci_mutex);
}