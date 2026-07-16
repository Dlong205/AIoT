#pragma once
#include "esp_err.h"
#include "rescue_types.h"

esp_err_t display_driver_init(void);

/* Vẽ toàn bộ màn hình trạng thái: state hiện tại, %pin, và nếu đang
 * SYS_STATE_RETURN thì vẽ thêm mũi tên hướng quay lại (heading_to_target_deg). */
esp_err_t display_driver_draw_status(system_state_t state,
                                      uint8_t battery_pct,
                                      bool show_return_arrow,
                                      float heading_to_target_deg);

/* Giới hạn tần suất update để tiết kiệm điện (OLED tốn hơn E-Ink khi refresh
 * liên tục) - gọi hàm này trước khi vẽ, nó tự quyết định có bỏ qua frame không. */
bool display_driver_should_update(uint32_t min_interval_ms);
