# C:/w/hello_world/components/BSEC2-Lib/CMakeLists.txt

# This component integrates the Bosch BSEC2 Library (pre-compiled binary)
idf_component_register(
    SRCS ""  # No C/C++ source files from the BSEC2 library itself to compile, as it's a binary.
             # If you plan to compile bsec2.cpp, uncomment the line below and adjust SRCS.
             # "bsec2.cpp"
    INCLUDE_DIRS
        .  # This tells the compiler to look in the current directory (BSEC2-Lib/) for headers like bsec2.h
    PRIV_REQUIRES
        "bme68x"  # The BSEC2 library depends on the raw BME68x Sensor API
        "esp_log" # For BSEC related logging
        "driver"  # As BSEC internally uses the BME68x API, which needs the driver
)

# Link the pre-compiled static library for ESP32
# Ensure this path is correct relative to this CMakeLists.txt file.
target_link_libraries(${COMPONENT_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/esp32/libalgobsec.a
)