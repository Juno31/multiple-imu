idf_component_register(
    SRCS         "bno085.c"
    INCLUDE_DIRS "include"
    REQUIRES     esp_driver_i2c log freertos esp_timer
)
