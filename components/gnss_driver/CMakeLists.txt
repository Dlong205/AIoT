idf_component_register(
    SRCS "gnss_driver.c" "nmea_parser.c"   # giữ nguyên SRCS hiện có của bạn
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_uart esp_driver_gpio
    PRIV_REQUIRES main
)