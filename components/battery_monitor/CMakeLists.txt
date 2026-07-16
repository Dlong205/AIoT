idf_component_register(
    SRCS "battery_monitor.c"
    INCLUDE_DIRS "include"
    REQUIRES common esp_adc
    PRIV_REQUIRES main
)
