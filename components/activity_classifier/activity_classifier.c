#include "activity_classifier.h"
#include <math.h>
#include <string.h>

#define WINDOW_SIZE 20   /* ~0.4s @ 50Hz, tinh chỉnh theo thực nghiệm */

/* TODO: tinh chỉnh các ngưỡng này bằng dữ liệu thật thu thập từ thiết bị */
#define TH_STILL_MAG_MAX      1.15f   /* g */
#define TH_FREEFALL_MAG_MAX   0.35f   /* g, gần 0g nghĩa là đang rơi tự do */
#define TH_IMPACT_MAG_MIN     2.5f    /* g, spike va chạm */

static float s_window[WINDOW_SIZE];
static int   s_idx = 0;
static bool  s_window_full = false;

void activity_classifier_init(void)
{
    memset(s_window, 0, sizeof(s_window));
    s_idx = 0;
    s_window_full = false;
}

activity_class_t activity_classifier_update(const imu_sample_t *sample, uint8_t *confidence_out)
{
    float mag = sqrtf(sample->ax * sample->ax +
                       sample->ay * sample->ay +
                       sample->az * sample->az);

    s_window[s_idx] = mag;
    s_idx = (s_idx + 1) % WINDOW_SIZE;
    if (s_idx == 0) s_window_full = true;

    /* TODO: thay bằng logic thật —
     * 1. Tính variance/std-dev của s_window để phân biệt STILL/WALKING/RUNNING.
     * 2. Quét window tìm pattern free-fall (mag < TH_FREEFALL_MAG_MAX) theo
     *    sau bởi impact (mag > TH_IMPACT_MAG_MIN) trong khoảng vài trăm ms
     *    -> ACTIVITY_FALL, confidence cao.
     * 3. Impact không có free-fall trước, kèm gyro thấp ngay sau đó (thiết
     *    bị nằm im dù người có thể vẫn đứng) -> ACTIVITY_DEVICE_DROPPED.
     * Đây chỉ là khung threshold ban đầu, không phải bản triển khai đầy đủ.
     */
    (void)mag;

    if (confidence_out) *confidence_out = 50; /* placeholder */
    return ACTIVITY_STILL;
}
