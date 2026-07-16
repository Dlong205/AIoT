idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        common
        imu_driver
        mag_driver
        baro_driver
        gnss_driver
        lora_driver
        display_driver
        activity_classifier
        pdr
        state_machine
        battery_monitor
        power_mgmt
        mesh_protocol
        driver
        esp_driver_gpio
        esp_driver_i2c
        nvs_flash
        esp_timer
)