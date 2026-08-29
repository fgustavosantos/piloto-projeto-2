idf_component_register(
    SRCS "main.c"
    REQUIRES esp_driver_twai esp_timer
    INCLUDE_DIRS "."
)
