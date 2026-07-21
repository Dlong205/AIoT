idf_component_register(
    SRCS "imu_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES common esp_driver_i2c esp_timer
    PRIV_REQUIRES main
)