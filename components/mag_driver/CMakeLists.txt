idf_component_register(
    SRCS "mag_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES common driver
    PRIV_REQUIRES main
)
