#pragma once
#include "esp_err.h"

esp_err_t baro_driver_init(void);

/* Đọc áp suất (Pa) và nhiệt độ (°C) đã compensate theo calib data trên chip. */
esp_err_t baro_driver_read(float *out_pressure_pa, float *out_temperature_c);

/* Ước lượng độ cao (m) từ áp suất, so với sea_level_pa tham chiếu
 * (mặc định dùng 101325.0f nếu không biết áp suất mực nước biển tại chỗ). */
float baro_driver_altitude_m(float pressure_pa, float sea_level_pa);