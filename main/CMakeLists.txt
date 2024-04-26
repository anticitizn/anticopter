set(requires "")
idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    list(APPEND requires esp_stubs esp-tls esp_http_server protocol_examples_common nvs_flash esp_psram)
endif()

idf_component_register(SRCS "led_strip/led_strip_encoder.c" "lsm6dsr/lsm6dsr_reg.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${requires})
