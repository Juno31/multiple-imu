idf_component_register(
    SRCS         "main.c"
    INCLUDE_DIRS "."
    REQUIRES     log freertos esp_timer esp_wifi nvs_flash esp_netif
)
