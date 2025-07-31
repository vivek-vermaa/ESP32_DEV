
/*
 * Main.c
 *
 * Description:
 *   Main application for measuring Air Quality Index using the Bosch BME680 sensor.
 *   Built on FreeRTOS for efficient task management and real-time performance.
 *
 * Author:
 *  C) Vivek Verma (tovivekverma@hotmail.com) 2025- 2030
 *
 * License:
 *   This software is released under the MIT License.
 *   You are free to use, modify, and distribute this software with proper attribution.
 *
 * Disclaimer:
 *   Provided as-is without any warranty. Use at your own risk.
 
     Open Source Open Mind 
	 
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h> // For NAN
#include "esp_log.h" // For ESP_LOGE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"

// If MY_EVENT_BASE is not used elsewhere in your project, consider removing it
ESP_EVENT_DECLARE_BASE(MY_EVENT_BASE);
ESP_EVENT_DEFINE_BASE(MY_EVENT_BASE);


#include "esp_timer.h"
#include "driver/spi_master.h"
#include "bme680.h"
#include "bsec_integration.h"
#include "bsec_datatypes.h" // For BSEC_OUTPUT_IAQ, BSEC_OUTPUT_CO2_EQUIVALENT etc.
#include "bsec_interface.h" // This usually declares bsec_do_steps, bsec_init, etc.

// --- START: Temporary/Fallback for BSEC_MAX_INPUTS ---
// If BSEC_MAX_INPUTS is still undeclared after including bsec_datatypes.h and bsec_interface.h,
// uncomment and use a sensible fallback value. Typically around 5-10 inputs are sufficient for BME680.
#ifndef BSEC_MAX_INPUTS
#define BSEC_MAX_INPUTS 10 // A safe value if not defined by your BSEC headers
#endif
// --- END: Temporary/Fallback for BSEC_MAX_INPUTS ---


// SPI pins & host
#define SPI_HOST       SPI2_HOST // Changed from HSPI_HOST to SPI2_HOST for newer ESP-IDF versions
#define PIN_NUM_MISO   19
#define PIN_NUM_MOSI   23 // Corrected typo here (PIN_MOSI -> PIN_NUM_MOSI)
#define PIN_NUM_CLK    18
#define PIN_NUM_CS     5

ESP_EVENT_DEFINE_BASE(BME680_EVENT);

typedef enum {
    BME680_EVENT_MEASURE_TRIGGER,
    BME680_EVENT_DATA_READY,
} bme680_event_id_t;

// Structure to carry sensor + BSEC outputs
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint32_t gas_resistance;
    float iaq;       // Indoor Air Quality index (0–1000)
    float co2_eq;    // CO₂‐equivalent (ppm)
    float sensor_stablization; //check if sensor is stabilized
} bme680_evt_data_t;

static struct bme680_dev sensor;
static spi_device_handle_t spi_handle; // Make spi_handle accessible globally or pass it

// Forward declarations of static functions
static void measure_timer_cb(void *arg);
static void measure_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data);
static void data_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data);

// SPI user read/write functions for BME680 driver
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    spi_transaction_t t = {
        .flags = 0,
        .cmd = 0,
        .addr = reg_addr,
        .length = len * 8, // Length in bits
        .rxlength = len * 8, // Receive length
        .tx_buffer = NULL,
        .rx_buffer = reg_data,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    return (ret == ESP_OK) ? BME680_OK : BME680_E_COM_FAIL;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    spi_transaction_t t = {
        .flags = 0,
        .cmd = 0,
        .addr = reg_addr,
        .length = len * 8, // Length in bits
        .tx_buffer = reg_data,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    return (ret == ESP_OK) ? BME680_OK : BME680_E_COM_FAIL;
}

static esp_err_t init_spi_and_bme680(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI, // Corrected here too
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000,  // 5 MHz
        .mode = 0, // SPI mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle));

    sensor.dev_id = 0;
    sensor.intf = BME680_SPI_INTF;
    sensor.read = user_spi_read;
    sensor.write = user_spi_write;
    sensor.delay_ms = vTaskDelay;

   int8_t rslt = bme680_init(&sensor);
   return (rslt == BME680_OK) ? ESP_OK : ESP_FAIL;
  
  return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(init_spi_and_bme680());
    bsec_init();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(
        BME680_EVENT, BME680_EVENT_MEASURE_TRIGGER,
        measure_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        BME680_EVENT, BME680_EVENT_DATA_READY,
        data_event_handler, NULL));

    // Start periodic timer
    const esp_timer_create_args_t timer_args = {
        .callback = measure_timer_cb,
        .name = "bme680_timer"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 5 * 1000 * 1000)); // 5 seconds in microseconds
}

// Called from ISR context.
static void IRAM_ATTR measure_timer_cb(void *arg)
{
    esp_event_isr_post(
        BME680_EVENT,
        BME680_EVENT_MEASURE_TRIGGER,
        NULL,
        0,
        NULL
    );
}

// Fires on measure event, runs BME680, then BSEC
static void measure_event_handler(
    void *handler_arg,
    esp_event_base_t base, int32_t id,
    void *event_data)
{ // <--- *** THIS OPENING CURLY BRACE IS ESSENTIAL! ***
    // 1) Trigger forced measurement
    sensor.power_mode = BME680_FORCED_MODE;
    bme680_set_sensor_mode(&sensor);

    // 2) Wait required duration
    uint32_t duration_ms =1000;
  //  bme680_get_profile_dur(&duration_ms, &sensor);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // 3) Read raw data
    struct bme680_field_data raw;
    //bme680_get_sensor_data(&raw, &sensor);

    // 4) Prepare inputs for BSEC
    bsec_input_t inputs[BSEC_MAX_INPUTS];
    uint8_t n_inputs = 0;
    int64_t timestamp_us = esp_timer_get_time();

    // Add sensor data to inputs array
    inputs[n_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
    //inputs[n_inputs].signal = raw.temperature;
    inputs[n_inputs].time_stamp = timestamp_us;
    n_inputs++;

    inputs[n_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
  //  inputs[n_inputs].signal = raw.humidity;
    inputs[n_inputs].time_stamp = timestamp_us;
    n_inputs++;

    inputs[n_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
   // inputs[n_inputs].signal = (float)raw.gas_resistance; // Cast to float
    inputs[n_inputs].time_stamp = timestamp_us;
    n_inputs++;

    // 5) Run BSEC to get IAQ & CO2‐eq
    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t n_outputs = BSEC_NUMBER_OUTPUTS;

    bsec_library_return_t bsec_status = bsec_do_steps(inputs, n_inputs, outputs, &n_outputs);

    if (bsec_status != BSEC_OK) {
        ESP_LOGE("BME680", "BSEC processing failed with status: %d", bsec_status);
        return;
    }

    // 6) Package & post DATA event
    bme680_evt_data_t *evt = malloc(sizeof(*evt));
    if (evt == NULL) {
        ESP_LOGE("BME680", "Failed to allocate memory for event data");
        return;
    }
   // evt->temperature   = raw.temperature;
   // evt->humidity      = raw.humidity;
   // evt->pressure      = raw.pressure;
   // evt->gas_resistance= raw.gas_resistance;
    evt->iaq           = NAN;
    evt->sensor_stablization        = NAN;

    for (int i = 0; i < n_outputs; i++)
    {
        if (outputs[i].sensor_id == BSEC_OUTPUT_IAQ_ESTIMATE) {
            evt->iaq = outputs[i].signal;
        }
        if (outputs[i].sensor_id == BSEC_OUTPUT_STABILIZATION_STATUS) {
            evt->sensor_stablization = outputs[i].signal;
        }
    }

    esp_event_post(
        BME680_EVENT, BME680_EVENT_DATA_READY,
        evt, sizeof(*evt),
        portMAX_DELAY);
} // <--- The corresponding closing brace for measure_event_handler

static void data_event_handler(
    void *handler_arg,
    esp_event_base_t base, int32_t id,
    void *event_data)
{
    bme680_evt_data_t *d = (bme680_evt_data_t *)event_data;

    printf("T: %.2f °C, H: %.2f %%, P: %.2f hPa\n",
           d->temperature,
           d->humidity,
           d->pressure / 100.0f);

    printf("GasRes: %lu Ω, IAQ: %.0f, CO2‐eq: %.0f ppm\n\n",
           d->gas_resistance,
           d->iaq,
           d->sensor_stablization);

    free(d);
}