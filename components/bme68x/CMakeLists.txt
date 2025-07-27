# C:/w/hello_world/components/bme68x/CMakeLists.txt

# This component provides the BME68x Sensor API
idf_component_register(
    SRCS
        src/bme68x/bme68x.c
        src/bme68x/bme68x_defs.c
    INCLUDE_DIRS
        src/bme68x  # This tells the compiler where to find bme68x.h and bme68x_defs.h
    PRIV_REQUIRES
        "esp_log"   # Standard ESP-IDF logging library
        "driver"    # Includes low-level I2C/SPI drivers used by the sensor API
)