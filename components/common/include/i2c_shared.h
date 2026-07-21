#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"

extern i2c_master_bus_handle_t i2c0_bus;
extern i2c_master_bus_handle_t i2c1_bus;

esp_err_t i2c_shared_init(void);
