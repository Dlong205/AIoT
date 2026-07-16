#pragma once
#include "esp_err.h"
#include "rescue_types.h"

typedef struct {
    bool    has_fix;
    double  lat;
    double  lon;
    float   hdop;          /* độ chính xác ngang, càng thấp càng tốt */
    uint8_t satellites;
    int64_t timestamp_us;
} gnss_fix_t;

/* Khởi tạo UART + GPIO điều khiển nguồn cho GY-NEO7M. */
esp_err_t gnss_driver_init(void);

/* Bật/tắt module qua load-switch (GNSS_POWER_EN_GPIO) để tiết kiệm pin
 * khi ở SYS_STATE_IDLE hoặc đã có P0 tin cậy. */
esp_err_t gnss_driver_power(bool on);

/* Blocking, gọi trong gnss_task: đọc 1 dòng NMEA, parse, trả về fix.
 * Trả ESP_ERR_TIMEOUT nếu không có dữ liệu mới trong timeout_ms. */
esp_err_t gnss_driver_read_fix(gnss_fix_t *out_fix, uint32_t timeout_ms);
