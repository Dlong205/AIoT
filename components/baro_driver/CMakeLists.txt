idf_component_register(
    SRCS "baro_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES common esp_driver_i2c
    PRIV_REQUIRES main
)