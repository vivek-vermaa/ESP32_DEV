#include <stdio.h>

#include <stdint.h>

#include <string.h>

#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "driver/i2c.h"

#include "esp_log.h"

#include "bme680.h" // Your header

#include "portmacro.h"




#define I2C_MASTER_NUM I2C_NUM_0

#define I2C_MASTER_SCL_IO 17

#define I2C_MASTER_SDA_IO 18

#define I2C_MASTER_FREQ_HZ 100000

#define BME680_I2C_ADDR 0x77 // Use 0x77 if SDO is high




static const char* TAG = "BME680";




static void i2c_master_init(void) {

i2c_config_t conf = {

.mode = I2C_MODE_MASTER,

.sda_io_num = I2C_MASTER_SDA_IO,

.sda_pullup_en = GPIO_PULLUP_ENABLE,

.scl_io_num = I2C_MASTER_SCL_IO,

.scl_pullup_en = GPIO_PULLUP_ENABLE,

.master.clk_speed = I2C_MASTER_FREQ_HZ,

};

ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

}




// I2C read/write function prototypes for bme680_dev

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

void user_delay_ms(uint32_t period);




void app_main(void)

{

i2c_master_init();




struct bme680_dev gas_sensor;

int8_t rslt;




// Set BME680 device parameters

gas_sensor.dev_id = BME680_I2C_ADDR;

gas_sensor.intf = BME680_I2C_INTF;

gas_sensor.read = user_i2c_read;

gas_sensor.write = user_i2c_write;

gas_sensor.delay_ms = user_delay_ms;

gas_sensor.amb_temp = 25; // ambient temp for gas calculations




rslt = bme680_init(&gas_sensor);

if(rslt != BME680_OK){

ESP_LOGE(TAG, "BME680 init failed (%d)", rslt);

return;

}




// Set sensor settings

uint16_t settings_sel = BME680_OST_SEL | BME680_OSH_SEL | BME680_OSP_SEL | BME680_FILTER_SEL | BME680_GAS_MEAS_SEL;

gas_sensor.tph_sett.os_hum = BME680_OS_2X;

gas_sensor.tph_sett.os_temp = BME680_OS_4X;

gas_sensor.tph_sett.os_pres = BME680_OS_4X;

gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;

gas_sensor.gas_sett.heatr_temp = 320; // 320°C gas sensor heater

gas_sensor.gas_sett.heatr_dur = 150; // 150 ms heater on




rslt = bme680_set_sensor_settings(settings_sel, &gas_sensor);




gas_sensor.power_mode = BME680_FORCED_MODE;




while (1) {

rslt = bme680_set_sensor_mode(&gas_sensor);

if(rslt != BME680_OK) {

ESP_LOGE(TAG, "Failed to set sensor mode (%d)", rslt);

vTaskDelay(pdMS_TO_TICKS(1000));

continue;

}




// Wait for measurement completion

uint16_t meas_dur = 0;

bme680_get_profile_dur(&meas_dur, &gas_sensor);

vTaskDelay(pdMS_TO_TICKS(meas_dur));




struct bme680_field_data data;

rslt = bme680_get_sensor_data(&data, &gas_sensor);

if(rslt == BME680_OK) {

ESP_LOGI(TAG, "Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa, Gas: %ld ohms",

data.temperature / 100.0f,

data.humidity / 1000.0f,

data.pressure / 100.0f,

data.gas_resistance);

} else {

ESP_LOGE(TAG, "Read error (%d)", rslt);

}

vTaskDelay(pdMS_TO_TICKS(2000));

}

}




// ---- Implementation of I2C and delay functions ----




int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)

{

i2c_cmd_handle_t cmd = i2c_cmd_link_create();

i2c_master_start(cmd);

i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true); // 7-bit + W

i2c_master_write_byte(cmd, reg_addr, true);

i2c_master_start(cmd);

i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true);

i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);

i2c_master_stop(cmd);

esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

i2c_cmd_link_delete(cmd);

return (ret == ESP_OK) ? 0 : -1;

}




int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)

{

i2c_cmd_handle_t cmd = i2c_cmd_link_create();

i2c_master_start(cmd);

i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);

i2c_master_write_byte(cmd, reg_addr, true);

i2c_master_write(cmd, data, len, true);

i2c_master_stop(cmd);

esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

i2c_cmd_link_delete(cmd);

return (ret == ESP_OK) ? 0 : -1;

}




void user_delay_ms(uint32_t period)

{

vTaskDelay(period / portTICK_PERIOD_MS);

}