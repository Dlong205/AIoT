idf_component_register(
    SRCS "display_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c
    PRIV_REQUIRES main
)