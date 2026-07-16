#pragma once
#include "rescue_types.h"

/* Reset vị trí tương đối về (0,0) - gọi khi vừa có P0 mới từ GNSS. */
void pdr_reset(void);

/* Gọi mỗi khi activity_classifier phát hiện 1 bước (EVT_STEP_DETECTED).
 * heading_deg lấy từ mag_driver_read_heading(). Cập nhật vị trí tương đối
 * nội bộ và tự động lưu breadcrumb nếu đủ điều kiện khoảng cách/góc rẽ
 * (xem BREADCRUMB_MIN_DISTANCE_M / BREADCRUMB_MIN_HEADING_DEG). */
void pdr_on_step(float heading_deg);

/* Lấy vị trí tương đối hiện tại so với P0. */
position_t pdr_get_position(void);

/* Số breadcrumb đã lưu, và lấy breadcrumb thứ i (0 = P0, tăng dần theo thời gian). */
int pdr_get_breadcrumb_count(void);
breadcrumb_t pdr_get_breadcrumb(int index);

/* Tính hướng (độ) và khoảng cách (m) cần đi để quay lại breadcrumb gần nhất
 * chưa đi qua, dùng trong SYS_STATE_RETURN để vẽ mũi tên trên OLED. */
void pdr_get_return_direction(float current_heading_deg,
                               float *out_heading_to_target_deg,
                               float *out_distance_m);
