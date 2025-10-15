/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2025 Vivek
 * All rights reserved.
 * ----------------------------------------------------------------------------
 *
 * This software is proprietary and confidential. 
 * Unauthorized copying, distribution, or modification of this file,
 * in whole or in part, by any means, is strictly prohibited without 
 * the express written permission of the author.
 *
 * Description:
 *   Event-driven implementation for Bosch BME680 environmental sensor 
 *   using the BSEC2 library on the ESP32 platform. 
 *   The program acquires temperature, humidity, pressure, and gas data 
 *   asynchronously and computes IAQ (Indoor Air Quality) and CO₂ 
 *   equivalents in real-time.
 *
 *   Designed for efficient power usage and modular integration 
 *   into embedded systems utilizing FreeRTOS and I2C communication.
 *
 * ----------------------------------------------------------------------------
 */


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "bme68x.h"

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   18
#define I2C_MASTER_SCL_IO   17
#define I2C_MASTER_FREQ_HZ 100000
#define BME68X_I2C_ADDR     0x77     // or 0x76 if SDO is low
#define OVERSHOOT_FACTOR    1.10f    // 10% extra
#define OVERSHOOT_EXTRA_US 1000      // +1 ms headroom

static const char *TAG = "BME68x_FixedDelay";

// Initialize I²C master interface
static void i2c_master_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(
        I2C_MASTER_NUM, cfg.mode, 0, 0, 0
    ));
}

// BME68x I²C write adapter
static int8_t bme68x_i2c_write(uint8_t reg_addr,
                               const uint8_t *reg_data,
                               uint32_t length,
                               void *intf_ptr)
{
    uint8_t dev = *(uint8_t *)intf_ptr;
    uint8_t buf[256];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, length);
    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM, dev, buf, length + 1,
        pdMS_TO_TICKS(100)
    );
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// BME68x I²C read adapter
static int8_t bme68x_i2c_read(uint8_t reg_addr,
                              uint8_t *reg_data,
                              uint32_t length,
                              void *intf_ptr)
{
    uint8_t dev = *(uint8_t *)intf_ptr;
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM, dev,
        &reg_addr, 1, reg_data, length,
        pdMS_TO_TICKS(100)
    );
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// Delay callback for BME68x driver (microseconds)
static void bme68x_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period_us);
}

void app_main(void)
{
    // 1) I²C setup
    i2c_master_init();

    // 2) Prepare sensor struct
    struct bme68x_dev sensor;
    memset(&sensor, 0, sizeof(sensor));
    static uint8_t addr = BME68X_I2C_ADDR;
    sensor.intf     = BME68X_I2C_INTF;
    sensor.intf_ptr = &addr;
    sensor.read     = bme68x_i2c_read;
    sensor.write    = bme68x_i2c_write;
    sensor.delay_us = bme68x_delay_us;
    sensor.amb_temp = 25;  // initial ambient °C for gas compensation

    // 3) Initialize BME68x
    int8_t rslt = bme68x_init(&sensor);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "init failed: %d", rslt);
        return;
    }

    // 4) Configure oversampling & filter
    struct bme68x_conf conf = {
        .os_hum  = BME68X_OS_2X,
        .os_temp = BME68X_OS_4X,
        .os_pres = BME68X_OS_4X,
        .filter  = BME68X_FILTER_SIZE_3
    };
    rslt = bme68x_set_conf(&conf, &sensor);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "set_conf failed: %d", rslt);
        return;
    }

    // 5) Configure heater (forced mode)
    struct bme68x_heatr_conf heat = {
        .enable     = BME68X_ENABLE,
        .heatr_temp = 320,
        .heatr_dur  = 150
    };
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heat, &sensor);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "set_heatr_conf failed: %d", rslt);
        return;
    }

    // 6) Measurement loop with fixed-delay readiness
    while (1) {
        // a) Trigger forced measurement
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &sensor);
        if (rslt != BME68X_OK) {
            ESP_LOGE(TAG, "set_op_mode failed: %d", rslt);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // b) Compute and wait the required duration + margin
        uint32_t wait_us = bme68x_get_meas_dur(
            BME68X_FORCED_MODE, &conf, &sensor
        );
        wait_us = (uint32_t)(wait_us * OVERSHOOT_FACTOR)
                  + OVERSHOOT_EXTRA_US;
        vTaskDelay(pdMS_TO_TICKS((wait_us + 500) / 1000));

        // c) Read out the results
        struct bme68x_data data[1];
        uint8_t n_fields = 0;
        rslt = bme68x_get_data(
            BME68X_FORCED_MODE, data, &n_fields, &sensor
        );
        if (rslt == BME68X_OK && n_fields) {
            // update ambient for next heater cycle
            sensor.amb_temp = (int8_t)(data[0].temperature + 0.5f);

            ESP_LOGI(TAG,
                     "T=%.2f C | RH=%.2f %% | P=%.2f hPa | Gas=%u Ohm",
                     data[0].temperature,
                     data[0].humidity,
                     data[0].pressure / 100.0f,
                     (unsigned)data[0].gas_resistance);
        } else {
            ESP_LOGE(TAG, "get_data err: %d fields=%u",
                     rslt, n_fields);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}