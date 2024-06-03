
#ifndef SHTC3_SENSOR_H
#define SHTC3_SENSOR_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

// I2C Configuration Parameters
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define SHTC3_SENSOR_ADDR 0x70
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

// Define I2C Configuration Parameters here if needed

// Declare the read_temperature function
esp_err_t read_temperature(float *temperature);

// Declare the read_humidity function
esp_err_t read_humidity(float *humidity);

#endif // SHTC3_SENSOR_H
