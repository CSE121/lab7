#include "TempSensor.h"

static const char *TAG = "SHTC3";

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

uint8_t gencrc(uint8_t *data, size_t len) {
  uint8_t crc = 0xff;
  size_t i, j;
  for (i = 0; i < len; i++) {
    crc ^= data[i];
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }
  return crc;
}

esp_err_t read_temperature(float *temperature) {
  uint8_t data[3];
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | WRITE_BIT,
                        ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x7C, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0xA2, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }

  vTaskDelay(20 / portTICK_PERIOD_MS);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data[0], (i2c_ack_type_t)ACK_VAL);
  i2c_master_read_byte(cmd, &data[1], (i2c_ack_type_t)ACK_VAL);
  i2c_master_read_byte(cmd, &data[2], (i2c_ack_type_t)NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }

  uint8_t checksum = gencrc(data, 2);
  if (checksum != data[2]) {
    ESP_LOGE(TAG, "Temperature checksum error: expected 0x%02X, got 0x%02X",
             checksum, data[2]);
    return ESP_FAIL;
  }

  float temp_raw = (data[0] << 8) | data[1];
  *temperature = -45 + (175.0 * temp_raw / 65536.0);
  return ESP_OK;
}

esp_err_t read_humidity(float *humidity) {
  uint8_t data[3];
  esp_err_t ret;

  // Trigger humidity measurement
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | WRITE_BIT,
                        ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x5C, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x24, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }

  vTaskDelay(20 / portTICK_PERIOD_MS);

  // Read the sensor data
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data[0], (i2c_ack_type_t)ACK_VAL);
  i2c_master_read_byte(cmd, &data[1], (i2c_ack_type_t)ACK_VAL);
  i2c_master_read_byte(cmd, &data[2], (i2c_ack_type_t)NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }

  uint8_t checksum = gencrc(data, 2);
  if (checksum != data[2]) {
    ESP_LOGE(TAG, "humidity checksum error: expected 0x%02X, got 0x%02X",
             checksum, data[2]);
    return ESP_FAIL;
  }

  float humi_raw = (data[0] << 8) | data[1];
  *humidity = 100.0 * humi_raw / 65536.0;

  return ESP_OK;
}
