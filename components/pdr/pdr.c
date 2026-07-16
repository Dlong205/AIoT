#include "pdr.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

#define MAX_BREADCRUMBS 128
#define STEP_LENGTH_M   0.7f   /* TODO: hiệu chỉnh theo người dùng thực tế,
                                   hoặc ước lượng động từ cadence/accel */

static float s_x = 0, s_y = 0;
static float s_last_breadcrumb_x = 0, s_last_breadcrumb_y = 0;
static float s_last_heading = 0;
static breadcrumb_t s_breadcrumbs[MAX_BREADCRUMBS];
static int s_breadcrumb_count = 0;

void pdr_reset(void)
{
    s_x = s_y = 0;
    s_last_breadcrumb_x = s_last_breadcrumb_y = 0;
    s_breadcrumb_count = 0;
    /* Lưu P0 luôn là breadcrumb đầu tiên */
    if (MAX_BREADCRUMBS > 0) {
        s_breadcrumbs[0] = (breadcrumb_t){ .rel_x_m = 0, .rel_y_m = 0, .heading_deg = 0 };
        s_breadcrumb_count = 1;
    }
}

void pdr_on_step(float heading_deg)
{
    float rad = heading_deg * (float)M_PI / 180.0f;
    s_x += STEP_LENGTH_M * sinf(rad);
    s_y += STEP_LENGTH_M * cosf(rad);

    float dx = s_x - s_last_breadcrumb_x;
    float dy = s_y - s_last_breadcrumb_y;
    float dist = sqrtf(dx * dx + dy * dy);
    float heading_delta = fabsf(heading_deg - s_last_heading);
    if (heading_delta > 180.0f) heading_delta = 360.0f - heading_delta;

    bool should_save = (dist >= BREADCRUMB_MIN_DISTANCE_M) ||
                        (heading_delta >= BREADCRUMB_MIN_HEADING_DEG);

    if (should_save && s_breadcrumb_count < MAX_BREADCRUMBS) {
        s_breadcrumbs[s_breadcrumb_count++] = (breadcrumb_t){
            .rel_x_m = s_x, .rel_y_m = s_y, .heading_deg = heading_deg
        };
        s_last_breadcrumb_x = s_x;
        s_last_breadcrumb_y = s_y;
        /* TODO: nếu muốn bền vững qua reset (mất điện), ghi thêm breadcrumb
         * mới nhất vào NVS ở đây. */
    }
    s_last_heading = heading_deg;
}

position_t pdr_get_position(void)
{
    return (position_t){
        .is_absolute = false,
        .rel_x_m = s_x,
        .rel_y_m = s_y,
        .accuracy_m = -1, /* TODO: ước lượng sai số tích lũy PDR theo số bước đã đi */
    };
}

int pdr_get_breadcrumb_count(void) { return s_breadcrumb_count; }

breadcrumb_t pdr_get_breadcrumb(int index)
{
    if (index < 0 || index >= s_breadcrumb_count) {
        return (breadcrumb_t){0};
    }
    return s_breadcrumbs[index];
}

void pdr_get_return_direction(float current_heading_deg,
                               float *out_heading_to_target_deg,
                               float *out_distance_m)
{
    /* TODO: thuật toán đơn giản nhất: nhắm tới breadcrumb chưa "đi qua"
     * gần nhất theo thứ tự ngược (breadcrumb_count-1 -> 0), khi đến gần
     * (vd < 5m) thì chuyển sang breadcrumb kế tiếp. Ở đây tạm nhắm thẳng
     * về P0 (breadcrumb 0) để có bản chạy được đầu tiên. */
    if (s_breadcrumb_count == 0) {
        *out_heading_to_target_deg = current_heading_deg;
        *out_distance_m = 0;
        return;
    }
    float target_x = s_breadcrumbs[0].rel_x_m;
    float target_y = s_breadcrumbs[0].rel_y_m;
    float dx = target_x - s_x;
    float dy = target_y - s_y;
    *out_distance_m = sqrtf(dx * dx + dy * dy);
    *out_heading_to_target_deg = atan2f(dx, dy) * 180.0f / (float)M_PI;
    if (*out_heading_to_target_deg < 0) *out_heading_to_target_deg += 360.0f;
}
