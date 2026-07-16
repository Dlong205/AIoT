#pragma once
#include "esp_err.h"
#include "rescue_types.h"

esp_err_t power_mgmt_init(void);

/* Gọi mỗi khi state_machine chuyển trạng thái - áp dụng chính sách năng
 * lượng tương ứng (Bảng 5): 
 *   IDLE       -> chuẩn bị Deep Sleep, đánh thức bằng ngắt IMU (wake-on-motion)
 *   WALKING    -> full active, GNSS bật định kỳ để refresh P0
 *   SOS        -> full active bắt buộc, không được sleep
 *   RETURN     -> full active (cần IMU+mag liên tục để dẫn đường)
 *   LOW_POWER  -> giảm tần suất mọi task, tắt GNSS, giảm sample rate IMU
 */
void power_mgmt_on_state_change(system_state_t new_state);

/* Vào Deep Sleep thực sự, chỉ gọi khi chắc chắn đang ở IDLE và không có
 * SOS pending. MCU sẽ khởi động lại từ đầu khi wake (không giữ RAM). */
void power_mgmt_enter_deep_sleep(void);
