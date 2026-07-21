#include "state_machine.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* NOTE: các driver cụ thể (lora_driver, pdr, gnss_driver...) được include
 * và gọi trong main.c / các *_task riêng, không include trực tiếp ở đây để
 * giữ state_machine không phụ thuộc phần cứng - dễ unit test logic FSM.
 * Thay vào đó side-effect được thực hiện thông qua callback đăng ký từ main. */

static const char *TAG = "state_machine";
static QueueHandle_t s_event_queue = NULL;
static system_state_t s_state = SYS_STATE_IDLE;

void state_machine_init(void)
{
    s_event_queue = xQueueCreate(16, sizeof(app_event_t));
    s_state = SYS_STATE_IDLE;
}

void state_machine_post_event(app_event_t evt)
{
    if (s_event_queue) {
        xQueueSend(s_event_queue, &evt, 0);
    }
}

static void transition(system_state_t new_state)
{
    if (new_state != s_state) {
        ESP_LOGI(TAG, "state: %d -> %d", s_state, new_state);
        s_state = new_state;
    }
}

void state_machine_process(void)
{
    app_event_t evt;
    if (xQueueReceive(s_event_queue, &evt, pdMS_TO_TICKS(100)) != pdTRUE) {
        return; /* không có sự kiện mới trong 100ms, quay lại vòng lặp */
    }

    /* Bảng chuyển trạng thái theo Bảng 5 trong tài liệu dự án.
     * TODO: đây là khung sườn - cần bổ sung đầy đủ theo đúng bảng, và
     * gọi các side-effect thật (start SOS timeout timer, gnss_driver_power,
     * lora_driver_send_sos, pdr_reset...) thông qua callback/hàm ngoài. */
    switch (s_state) {
    case SYS_STATE_IDLE:
        if (evt.type == EVT_STEP_DETECTED) transition(SYS_STATE_WALKING);
        else if (evt.type == EVT_FALL_SUSPECTED) transition(SYS_STATE_FALL_SUSPECTED);
        else if (evt.type == EVT_BTN_SOS_PRESSED) transition(SYS_STATE_SOS);
        break;

    case SYS_STATE_WALKING:
        if (evt.type == EVT_FALL_SUSPECTED) transition(SYS_STATE_FALL_SUSPECTED);
        else if (evt.type == EVT_BTN_SOS_PRESSED) transition(SYS_STATE_SOS);
        else if (evt.type == EVT_RETURN_REQUESTED) transition(SYS_STATE_RETURN);
        /* TODO: quay về IDLE sau N giây không có STEP_DETECTED. */
        break;

    case SYS_STATE_FALL_SUSPECTED:
        /* TODO: chờ SOS_CONFIRM_TIMEOUT_MS cho người dùng bấm Cancel.
         * Cần 1 esp_timer riêng bắt đầu khi vào trạng thái này, huỷ nếu
         * nhận EVT_BTN_CANCEL_PRESSED trước timeout. */
        if (evt.type == EVT_BTN_CANCEL_PRESSED) transition(SYS_STATE_WALKING);
        else if (evt.type == EVT_FALL_CONFIRMED) transition(SYS_STATE_SOS);
        break;

    case SYS_STATE_SOS:
        /* TODO: gửi lại gói SOS định kỳ (heartbeat) cho tới khi được
         * cứu hộ xác nhận (ACK) hoặc pin hết. */
        if (evt.type == EVT_BTN_CANCEL_PRESSED) transition(SYS_STATE_WALKING);
        break;

    case SYS_STATE_RETURN:
        if (evt.type == EVT_RETURN_ARRIVED) transition(SYS_STATE_IDLE);
        else if (evt.type == EVT_FALL_SUSPECTED) transition(SYS_STATE_FALL_SUSPECTED);
        break;

    case SYS_STATE_LOW_POWER:
        /* TODO: giảm heartbeat interval, tắt GNSS, giảm sample rate IMU. */
        if (evt.type == EVT_BTN_SOS_PRESSED) transition(SYS_STATE_SOS);
        break;
    }

    if (evt.type == EVT_BATTERY_LOW && s_state != SYS_STATE_SOS) {
        transition(SYS_STATE_LOW_POWER);
    }
}

system_state_t state_machine_get_state(void)
{
    return s_state;
}
