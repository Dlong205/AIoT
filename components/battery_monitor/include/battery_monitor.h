#pragma once
#include "esp_err.h"

esp_err_t battery_monitor_init(void);

/* Đọc điện áp pin thực tế (sau khi nhân lại BATTERY_DIVIDER_RATIO), đơn vị Volt. */
float battery_monitor_read_voltage(void);

/* Ước lượng % pin dựa trên điện áp - CHỈ mang tính tương đối vì không có
 * fuel gauge (IC như MAX17048) như bản hoàn chỉnh. KHÔNG dùng số này để
 * tính "estimated runtime" chính xác, chỉ nên hiển thị dạng thanh/khoảng. */
uint8_t battery_monitor_read_percent(void);
