/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdint.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "endian.h"

#include "freertos/FreeRTOS.h"
#include "portmacro.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_NUM 0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS 1000

#define SCD41_SENSOR_ADDR 0x62 /*!< Slave address of the SCD41 sensor */

static esp_err_t scd41_stop_periodic_measurements(void)
{
  int ret;
  uint8_t write_buf[2] = {0x3f, 0x86};
  ret = i2c_master_write_to_device(I2C_MASTER_NUM, SCD41_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  return ret;
}

static esp_err_t scd41_start_periodic_measurements(void)
{
  int ret;
  uint8_t write_buf[2] = {0x21, 0xb1};
  ret = i2c_master_write_to_device(
      I2C_MASTER_NUM,
      SCD41_SENSOR_ADDR,
      write_buf,
      sizeof(write_buf),
      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  return ret;
}

static bool scd41_get_data_ready_status(void)
{
  int ret;
  uint8_t write_buf[2] = {0xe4, 0xb8};
  uint8_t read_buf[3] = {0, 0, 0};

  ret = i2c_master_write_read_device(
      I2C_MASTER_NUM,
      SCD41_SENSOR_ADDR,
      write_buf,
      sizeof(write_buf),
      read_buf,
      sizeof(read_buf),
      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  ESP_ERROR_CHECK(ret);
  // the old check is difficult to understand, prefer the newer one.
  // return ((read_buf[0] & 0x07) || (read_buf[1] != 0));

  uint16_t answer = be16toh(*(uint16_t *)read_buf);
  // check if the lower eleven bits are zero or not
  return (answer & 0x07ff);
}

static esp_err_t scd41_read_measurement(uint8_t *measurements, size_t msize)
{
  uint8_t cmd[] = {0xec, 0x05};
  return i2c_master_write_read_device(I2C_MASTER_NUM, SCD41_SENSOR_ADDR, cmd, sizeof(cmd), measurements, msize, 1000 / portTICK_PERIOD_MS);
}

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INT 0xff
// CRC calculation routine, as found in the scd41 datasheet
uint8_t sensirion_common_generate_crc(const uint8_t *data, uint16_t count)
{
  uint16_t current_byte;
  uint8_t crc = CRC8_INT;
  uint8_t crc_bit;
  for (current_byte = 0; current_byte < count; ++current_byte)
  {
    crc ^= (data[current_byte]);
    for (crc_bit = 8; crc_bit > 0; --crc_bit)
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ CRC8_POLYNOMIAL;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

static bool scd41_is_data_crc_correct(uint8_t *raw)
{
  for (int i = 0; i < 3; i++)
  {
    if (sensirion_common_generate_crc(&raw[i * 3], 3))
    {
      ESP_LOGE(TAG, "SCD41: CRC ERROR at word number %d\n", i);
      return false;
    }
  }
  return true;
}

static void scd41_calculate_and_show_data(uint8_t *raw)
{
  // if you get this far, you know that your raw measurements are 9 bytes
  // passing the array size here helps no one.
  uint16_t co2 = be16toh(*(uint16_t *)&raw[0]);
  uint16_t raw_temperature = be16toh(*(uint16_t *)&raw[3]);
  uint16_t raw_humidity = be16toh(*(uint16_t *)&raw[6]);

  // calculate temperature
  double temperature = -45 + 175 * (raw_temperature / (double)0xffff);
  double humidity = 100 * (raw_humidity / (double)0xffff);

  printf("SCD41: CO2: %d (ppm), temperature: %.02f C, humidity: %.02f %%\n", co2, temperature, humidity);
}

void scd41_poll()
{
  uint8_t raw_measurements[9];

  if (!scd41_get_data_ready_status())
  {
    printf("SCD41: no new data available\n");
    return;
  }

  ESP_ERROR_CHECK(scd41_read_measurement(raw_measurements, sizeof(raw_measurements)));

  if (!scd41_is_data_crc_correct(raw_measurements))
  {
    printf("SCD41: crc error!\n");
    return;
  }

  scd41_calculate_and_show_data(raw_measurements);
}

void scd41_init()
{
  ESP_ERROR_CHECK(scd41_stop_periodic_measurements());

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK(scd41_start_periodic_measurements());
  printf("SCD41: initialization finished\n");
}

void scd41_monitor(void)
{
  scd41_init();

  int count = 0;
  while (count < 15)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    scd41_poll();
    count++;
  }
}