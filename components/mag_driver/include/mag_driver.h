#pragma once
#include "esp_err.h"

esp_err_t mag_driver_init(void);

/* Trả về heading đã bù nghiêng (tilt-compensated) theo độ [0, 360).
 * Cần accel từ IMU để bù nghiêng chính xác khi người dùng không cầm phẳng. */
esp_err_t mag_driver_read_heading(float ax, float ay, float az, float *out_heading_deg);
