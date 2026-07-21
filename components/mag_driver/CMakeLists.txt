idf_component_register(
    SRCS "mag_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES common esp_driver_i2c
    PRIV_REQUIRES main
)
