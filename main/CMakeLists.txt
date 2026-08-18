set(requires "")
idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    list(APPEND requires esp_stubs esp-tls esp_http_server protocol_examples_common nvs_flash esp_psram)
endif()

idf_component_register(SRCS "led_strip/led_strip_encoder.c" 
                    "lsm6dsr/lsm6dsr_reg.c" "lsm6ds3/lsm6ds3_reg.c" "lis3mdl/lis3mdl_reg.c" 
                    "vl53l1x/vl53l1_platform.c" "vl53l1x/VL53L1X_api.c" "vl53l1x/VL53L1X_calibration.c"
                    "pmw3901/pmw3901.c"
                    "control/pid.c" 
                    "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${requires})
