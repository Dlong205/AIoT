#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "rescue_types.h"

esp_err_t display_driver_init(void);

esp_err_t display_driver_update(
    system_state_t state,
    uint8_t battery_pct,
    float heading_deg,
    float imu_ax, float imu_ay, float imu_az,
    float imu_gx, float imu_gy, float imu_gz,
    float altitude_m, float temperature_c,
    float pressure_pa,
    bool gnss_has_fix,
    bool mag_ok);

bool display_driver_should_update(uint32_t min_interval_ms);