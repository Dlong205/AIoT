idf_component_register(
    SRCS "power_mgmt.c"
    INCLUDE_DIRS "include"
    REQUIRES common esp_hw_support driver
    PRIV_REQUIRES main
)
