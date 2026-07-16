#pragma once
#include "rescue_types.h"

/* Reset bộ đệm/cửa sổ trượt nội bộ của classifier. */
void activity_classifier_init(void);

/* Nạp thêm 1 mẫu IMU vào cửa sổ trượt và trả về phân loại hiện tại.
 * confidence_out (0-100) dùng để đính kèm vào gói SOS.
 *
 * Thuật toán prototype (threshold, KHÔNG cần TFLite Micro):
 *  - accel magnitude ~1g kéo dài  -> STILL
 *  - accel magnitude dao động nhẹ, đều -> WALKING
 *  - accel magnitude dao động mạnh, đều, tần số cao -> RUNNING
 *  - free-fall ngắn (magnitude gần 0) rồi spike mạnh (>2.5-3g) -> FALL
 *  - spike mạnh nhưng không có free-fall trước đó, và sau đó vẫn tĩnh
 *    (không phải cử động người) -> DEVICE_DROPPED
 * Ngưỡng cụ thể cần tinh chỉnh bằng dữ liệu thu thập thực tế (mục 9/Bảng 14
 * trong tài liệu dự án - nâng cấp lên model nhẹ khi có dataset). */
activity_class_t activity_classifier_update(const imu_sample_t *sample, uint8_t *confidence_out);
