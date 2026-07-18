#pragma once
#include "esp_err.h"

esp_err_t mag_driver_init(void);

/* Trả về heading (bù nghiêng) theo độ [0, 360).
 * ax, ay, az: gia tốc từ IMU (tính bằng g hoặc m/s² — cần chuẩn hóa).
 * Nếu ax=ay=az=0 (không có IMU), heading tính không bù nghiêng. */
esp_err_t mag_driver_read_heading(float ax, float ay, float az, float *out_heading_deg);
