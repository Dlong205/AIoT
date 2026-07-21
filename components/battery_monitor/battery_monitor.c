#include "battery_monitor.h"
#include "app_config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "battery_monitor";
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/* Đường cong LiPo 1S đơn giản hoá: 4.20V=100%, 3.30V=0% (tuyến tính).
 * TODO: thay bằng bảng tra (lookup table) theo đường cong xả thực tế của
 * pin bạn dùng, vì LiPo không xả tuyến tính - sai số ADC + tuyến tính hoá
 * là lý do %pin ở prototype "chỉ mang tính tương đối" như đã lưu ý. */
#define BATT_V_FULL   4.20f
#define BATT_V_EMPTY  3.30f

esp_err_t battery_monitor_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = BATTERY_ADC_UNIT };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);
    ESP_LOGI(TAG, "ADC battery monitor init");
    return err;
}

float battery_monitor_read_voltage(void)
{
    if (!s_adc_handle) return BATT_V_FULL;
    int raw = 0;
    /* TODO: dùng adc_cali (ESP-IDF calibration API) thay vì quy đổi thô để
     * giảm sai số - vẫn sẽ có sai số vì không có fuel gauge chuyên dụng. */
    adc_oneshot_read(s_adc_handle, BATTERY_ADC_CHANNEL, &raw);
    float v_adc = (raw / 4095.0f) * 3.3f; /* giả định ADC 12-bit, Vref 3.3V */
    return v_adc * BATTERY_DIVIDER_RATIO;
}

uint8_t battery_monitor_read_percent(void)
{
    if (!s_adc_handle) return 100;
    float v = battery_monitor_read_voltage();
    if (v >= BATT_V_FULL) return 100;
    if (v <= BATT_V_EMPTY) return 0;
    return (uint8_t)(((v - BATT_V_EMPTY) / (BATT_V_FULL - BATT_V_EMPTY)) * 100.0f);
}
