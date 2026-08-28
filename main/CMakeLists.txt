idf_component_register(SRCS "transmit_sensor_can.c"
    REQUIRES esp_driver_twai esp_timer
    INCLUDE_DIRS ".")
