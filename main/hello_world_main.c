#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "bsec_interface.h"

// SPI config
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

#define BME680_SPI_HOST SPI3_HOST
#define BME680_SPI_SPEED_HZ 1000000

spi_device_handle_t bme680_handle;

// BSEC state
bsec_library_return_t bsec_status;

void output_ready(float temperature, float humidity, float pressure, float gas_resistance,
                  float iaq, int iaq_accuracy, float co2_equivalent, float breath_voc_equivalent)
{
    printf("Temp: %.2f °C |  Humidity: %.2f %% |  Pressure: %.2f hPa\n", temperature, humidity, pressure / 100.0f);
    printf("Gas: %.2f Ω   |  IAQ: %.2f (Acc: %d)\n", gas_resistance, iaq, iaq_accuracy);
    printf(" CO₂: %.2f ppm |  VOC: %.2f ppm\n\n", co2_equivalent, breath_voc_equivalent);
}

void init_spi()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = BME680_SPI_SPEED_HZ,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1
    };

    spi_bus_initialize(BME680_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(BME680_SPI_HOST, &devcfg, &bme680_handle);
}

esp_err_t bme680_spi_read(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .rxlength = (len + 1) * 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    uint8_t tx_buf[len + 1];
    tx_buf[0] = reg_addr | 0x80;  // Set read bit
    memset(&tx_buf[1], 0, len);

    t.tx_buffer = tx_buf;
    t.rx_buffer = data;

    return spi_device_transmit(bme680_handle, &t);
}

esp_err_t bme680_spi_write(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[len + 1];
    tx_buf[0] = reg_addr & 0x7F;  // Clear read bit
    memcpy(&tx_buf[1], data, len);

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buf
    };

    return spi_device_transmit(bme680_handle, &t);
}

void app_main()
{
    printf("🚀 Starting BSEC2 + BME680 SPI demo...\n");

    init_spi();

    // Initialize BSEC
    bsec_status = bsec_init();
    if (bsec_status != BSEC_OK) {
        printf("❌ BSEC init failed: %d\n", bsec_status);
        return;
    }

    // TODO: Initialize BME680 via SPI (load calibration data, set config)
    // TODO: Read raw data from BME680 via SPI
    // TODO: Feed raw data into BSEC2

    // For now, simulate values
    while (1) {
        float temperature = 24.5f;
        float humidity = 45.0f;
        float pressure = 101325.0f;
        float gas_resistance = 12000.0f;
        float iaq = 35.0f;
        int iaq_accuracy = 2;
        float co2 = 450.0f;
        float voc = 0.65f;

        output_ready(temperature, humidity, pressure, gas_resistance, iaq, iaq_accuracy, co2, voc);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}