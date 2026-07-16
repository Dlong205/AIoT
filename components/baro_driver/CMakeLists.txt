idf_component_register(
    SRCS "baro_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c
    PRIV_REQUIRES main
)