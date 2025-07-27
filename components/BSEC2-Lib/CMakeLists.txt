# C:/w/hello_world/components/BSEC2-Lib/CMakeLists.txt

idf_component_register(
    SRCS
        dummy.c  # Ensure dummy.c exists directly in components/BSEC2-Lib/
    INCLUDE_DIRS
        .        # Include the BSEC2-Lib/ folder for bsec2.h
    PRIV_REQUIRES
        "bme68x"
        "log"
        "driver"
)

target_link_libraries(${COMPONENT_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/esp32/libalgobsec.a
)