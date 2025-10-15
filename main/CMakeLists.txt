#idf_component_register(SRCS "hello_world_main.c"
#                       PRIV_REQUIRES spi_flash
#                       INCLUDE_DIRS "")
# C:/w/hello_world/main/CMakeLists.txt

# C:/w/hello_world/main/CMakeLists.txt
# This CMakeLists.txt defines the 'main' component of your project.

# C:/w/hello_world/main/CMakeLists.txt

idf_component_register(SRCS "hello_world_main.c"
                             "bsec_iaq.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES
                            "freertos"
                            "log"
                            # Changed from "spi" to "driver"
                            "driver"
                            "BSEC2-Lib"
                            "spi_flash"
                            "esp_event"
                            "esp_timer"
                            
)