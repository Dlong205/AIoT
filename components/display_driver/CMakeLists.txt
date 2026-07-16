idf_component_register(
    SRCS "display_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES common driver
    PRIV_REQUIRES main
)
