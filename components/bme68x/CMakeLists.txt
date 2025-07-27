# C:/w/hello_world/components/bme68x/CMakeLists.txt

#idf_component_register(
 #   SRCS
  #      src/bme68x/bme68x.c         # Correct path relative to component root
   #     src/bme68x/bme68x_defs.c    # Correct path relative to component root
    #INCLUDE_DIRS
     #   src/bme68x                  # Correct path for headers relative to component root
    #PRIV_REQUIRES
     #   "log"
      #  "driver"
       # "hal"
#)

# --- CORRECT ---
idf_component_register(SRCS "src/bme68x.c"
                       INCLUDE_DIRS "src")