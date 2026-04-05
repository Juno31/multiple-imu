idf_component_register(
    SRCS         "main.c"
    INCLUDE_DIRS "."
    REQUIRES     esp_driver_i2c log freertos esp_timer bno085 esp_wifi nvs_flash esp_netif
)
