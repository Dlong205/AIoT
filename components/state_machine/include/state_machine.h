#pragma once
#include "rescue_types.h"

void state_machine_init(void);

/* Nạp 1 sự kiện vào FSM. Được gọi từ imu_task, gnss_task, nút bấm ISR, v.v.
 * Nên đưa qua FreeRTOS queue rồi xử lý tuần tự trong state_machine_task,
 * KHÔNG gọi trực tiếp từ ISR. */
void state_machine_post_event(app_event_t evt);

/* Chạy 1 vòng xử lý: lấy sự kiện từ queue, cập nhật trạng thái, thực hiện
 * side-effect tương ứng (gọi lora_driver_send_sos, display update...).
 * Gọi trong vòng lặp của state_machine_task. */
void state_machine_process(void);

system_state_t state_machine_get_state(void);
