file(GLOB ESP32CAMERA "./components/esp32-camera/driver/private_include/*.h")

idf_component_register(SRCS
    "main.c"
    "motor_foc.c"
    "as5600.c"
    "lowside_current_sense.c"
    INCLUDE_DIRS
    "."

    PRIV_REQUIRES nvs_flash esp_psram esp_timer
    REQUIRES esp_adc
    )