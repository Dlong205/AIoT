#include "display_driver.h"
#include "app_config.h"
#include "i2c_shared.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "display_driver";
static i2c_master_dev_handle_t s_oled_dev = NULL;
static int64_t s_last_update_us = 0;
static int64_t s_last_toggle_us = 0;
static int s_screen_index = 0;

#define DISPLAY_SCREEN_COUNT 3
#ifndef DISPLAY_SCREEN_INTERVAL_MS
#define DISPLAY_SCREEN_INTERVAL_MS 10000
#endif

#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_PAGES  (OLED_HEIGHT / 8)
#define FB_SIZE     (OLED_WIDTH * OLED_PAGES)
#define FONT_W      5
#define FONT_H      7
#define CHAR_W      (FONT_W + 1)
#define LINE_H      (FONT_H + 1)

static uint8_t s_fb[FB_SIZE];

static esp_err_t oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_transmit(s_oled_dev, buf, 2, 100);
}

static esp_err_t oled_data_bulk(const uint8_t *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        size_t chunk = len - sent;
        if (chunk > 127) chunk = 127;
        uint8_t buf[128];
        buf[0] = 0x40;
        memcpy(buf + 1, data + sent, chunk);
        esp_err_t err = i2c_master_transmit(s_oled_dev, buf, chunk + 1, 500);
        if (err != ESP_OK) return err;
        sent += chunk;
    }
    return ESP_OK;
}

static void fb_clear(void) { memset(s_fb, 0, FB_SIZE); }

static void fb_set_pixel(int x, int y, int on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    if (on)
        s_fb[x + (y / 8) * OLED_WIDTH] |= (1 << (y & 7));
    else
        s_fb[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y & 7));
}

static void fb_fill_rect(int x, int y, int w, int h, int on)
{
    for (int yy = y; yy < y + h && yy < OLED_HEIGHT; yy++)
        for (int xx = x; xx < x + w && xx < OLED_WIDTH; xx++)
            fb_set_pixel(xx, yy, on);
}

static void fb_draw_line(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    for (;;) {
        fb_set_pixel(x0, y0, 1);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void fb_draw_circle(int cx, int cy, int r)
{
    int x = r, y = 0, err = 0;
    while (x >= y) {
        fb_set_pixel(cx + x, cy + y, 1);
        fb_set_pixel(cx + y, cy + x, 1);
        fb_set_pixel(cx - y, cy + x, 1);
        fb_set_pixel(cx - x, cy + y, 1);
        fb_set_pixel(cx - x, cy - y, 1);
        fb_set_pixel(cx - y, cy - x, 1);
        fb_set_pixel(cx + y, cy - x, 1);
        fb_set_pixel(cx + x, cy - y, 1);
        y += 1;
        err += 1 + 2 * y;
        if (2 * (err - x) + 1 > 0) {
            x -= 1;
            err += 1 - 2 * x;
        }
    }
}

#define OLED_COL_OFFSET  0

static esp_err_t fb_flush(void)
{
    esp_err_t err;
    for (int page = 0; page < OLED_PAGES; page++) {
        err = oled_cmd(0xB0 | page);
        if (err != ESP_OK) return err;
        err = oled_cmd(0x00 | (OLED_COL_OFFSET & 0x0F));
        if (err != ESP_OK) return err;
        err = oled_cmd(0x10 | (OLED_COL_OFFSET >> 4));
        if (err != ESP_OK) return err;
        err = oled_data_bulk(s_fb + page * OLED_WIDTH, OLED_WIDTH);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static const uint8_t s_font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00,
    0x00, 0x07, 0x00, 0x07, 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14,
    0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
    0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x41, 0x22, 0x1C, 0x00,
    0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x08, 0x08, 0x3E, 0x08, 0x08,
    0x00, 0x50, 0x30, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x00, 0x60, 0x60, 0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02,
    0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
    0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31,
    0x18, 0x14, 0x12, 0x7F, 0x10, 0x27, 0x45, 0x45, 0x45, 0x39,
    0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
    0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E,
    0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x56, 0x36, 0x00, 0x00,
    0x00, 0x08, 0x14, 0x22, 0x41, 0x14, 0x14, 0x14, 0x14, 0x14,
    0x41, 0x22, 0x14, 0x08, 0x00, 0x02, 0x01, 0x51, 0x09, 0x06,
    0x32, 0x49, 0x79, 0x41, 0x3E, 0x7E, 0x11, 0x11, 0x11, 0x7E,
    0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
    0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41,
    0x7F, 0x09, 0x09, 0x01, 0x01, 0x3E, 0x41, 0x41, 0x51, 0x32,
    0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
    0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41,
    0x7F, 0x40, 0x40, 0x40, 0x40, 0x7F, 0x02, 0x04, 0x02, 0x7F,
    0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
    0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E,
    0x7F, 0x09, 0x19, 0x29, 0x46, 0x46, 0x49, 0x49, 0x49, 0x31,
    0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
    0x1F, 0x20, 0x40, 0x20, 0x1F, 0x7F, 0x20, 0x18, 0x20, 0x7F,
    0x63, 0x14, 0x08, 0x14, 0x63, 0x03, 0x04, 0x78, 0x04, 0x03,
    0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00, 0x7F, 0x41, 0x41,
    0x02, 0x04, 0x08, 0x10, 0x20, 0x41, 0x41, 0x7F, 0x00, 0x00,
    0x04, 0x02, 0x01, 0x02, 0x04, 0x40, 0x40, 0x40, 0x40, 0x40,
    0x00, 0x01, 0x02, 0x04, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
    0x7F, 0x48, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x20,
    0x38, 0x44, 0x44, 0x48, 0x7F, 0x38, 0x54, 0x54, 0x54, 0x18,
    0x08, 0x7E, 0x09, 0x01, 0x02, 0x08, 0x14, 0x54, 0x54, 0x3C,
    0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00,
    0x20, 0x40, 0x44, 0x3D, 0x00, 0x00, 0x7F, 0x10, 0x28, 0x44,
    0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
    0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38,
    0x7C, 0x14, 0x14, 0x14, 0x08, 0x08, 0x14, 0x14, 0x18, 0x7C,
    0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x20,
    0x04, 0x3F, 0x44, 0x40, 0x20, 0x3C, 0x40, 0x40, 0x20, 0x7C,
    0x1C, 0x20, 0x40, 0x20, 0x1C, 0x3C, 0x40, 0x30, 0x40, 0x3C,
    0x44, 0x28, 0x10, 0x28, 0x44, 0x0C, 0x50, 0x50, 0x50, 0x3C,
    0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x08, 0x36, 0x41, 0x00,
    0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x41, 0x36, 0x08, 0x00,
    0x08, 0x08, 0x2A, 0x1C, 0x08, 0x08, 0x1C, 0x2A, 0x08, 0x08,
};

static void draw_char(int x, int y, char c)
{
    if (c < 0x20 || c > 0x7E) c = ' ';
    const uint8_t *bmp = s_font5x7 + (c - 0x20) * 5;
    for (int col = 0; col < FONT_W; col++) {
        uint8_t bits = bmp[col];
        for (int row = 0; row < FONT_H; row++) {
            if (bits & (1 << row))
                fb_set_pixel(x + col, y + row, 1);
        }
    }
}

static void draw_str(int x, int y, const char *s)
{
    while (*s) {
        if (x + FONT_W > OLED_WIDTH) { x = 0; y += LINE_H; }
        draw_char(x, y, *s);
        x += CHAR_W;
        s++;
    }
}

static void draw_icon_battery(int x, int y, int pct)
{
    int w = 14, h = 6;
    fb_fill_rect(x + w, y + 1, 2, h - 2, 1);
    int bars = (pct + 10) / 20;
    if (bars > 5) bars = 5;
    int fill_w = (w - 2) * bars / 5;
    if (bars > 0)
        fb_fill_rect(x + 1, y + 1, fill_w, h - 2, 1);
    fb_fill_rect(x, y, w, 1, 1);
    fb_fill_rect(x, y + h - 1, w, 1, 1);
    fb_fill_rect(x, y + 1, 1, h - 2, 1);
    fb_fill_rect(x + w - 1, y + 1, 1, h - 2, 1);
}

static const char *state_name(system_state_t state)
{
    switch (state) {
        case SYS_STATE_IDLE:           return "IDLE";
        case SYS_STATE_WALKING:        return "WALKING";
        case SYS_STATE_FALL_SUSPECTED: return "FALL?";
        case SYS_STATE_SOS:            return "SOS!";
        case SYS_STATE_RETURN:         return "RETURN";
        case SYS_STATE_LOW_POWER:      return "LOW PWR";
        default:                       return "?";
    }
}

static void draw_icon_state(int x, int y, system_state_t state)
{
    fb_fill_rect(x, y, 9, 9, 1);
    fb_fill_rect(x + 1, y + 1, 7, 7, 0);
    draw_char(x + 2, y + 1, state_name(state)[0]);
}

static void draw_icon_gnss(int x, int y, int has_fix)
{
    fb_fill_rect(x + 3, y, 2, 8, 1);
    fb_fill_rect(x, y + 3, 8, 2, 1);
    if (!has_fix) {
        fb_fill_rect(x, y, 1, 1, 1);
        fb_fill_rect(x + 7, y, 1, 1, 1);
        fb_fill_rect(x, y + 7, 1, 1, 1);
        fb_fill_rect(x + 7, y + 7, 1, 1, 1);
    }
}

static const char *heading_to_cardinal(float heading_deg)
{
    static const char *dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int idx = ((int)((heading_deg + 22.5f) / 45.0f)) & 7;
    return dirs[idx];
}

static void draw_layout_status(system_state_t state, uint8_t batt,
                                float heading, float alt, float temp, float press_pa,
                                int gnss_fix, int mag_ok)
{
    draw_icon_state(0, 0, state);
    draw_str(12, 0, state_name(state));
    char batt_str[8];
    snprintf(batt_str, sizeof(batt_str), "%d%%", batt);
    int batt_x = OLED_WIDTH - CHAR_W * (int)strlen(batt_str);
    draw_icon_battery(batt_x - 18, 0, batt);
    draw_str(batt_x, 0, batt_str);
    char line1[24];
    snprintf(line1, sizeof(line1), "Huong: %.0fdeg %s%s", heading,
             heading_to_cardinal(heading), mag_ok ? "" : "(?)");
    draw_str(0, LINE_H, line1);
    char line2[24];
    snprintf(line2, sizeof(line2), "Ap suat: %.3f atm", press_pa / 101325.0f);
    draw_str(0, LINE_H * 2, line2);
    char line3[24];
    snprintf(line3, sizeof(line3), "Nhiet do: %.1f C", temp);
    draw_str(0, LINE_H * 3, line3);
    char line4[24];
    snprintf(line4, sizeof(line4), "Do cao: %.0f m", alt);
    draw_str(0, LINE_H * 4, line4);
    draw_icon_gnss(0, LINE_H * 5, gnss_fix);
    draw_str(10, LINE_H * 5, gnss_fix ? "GNSS: fix" : "GNSS: no fix");
}

static void draw_layout_raw(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             float press_pa, float temp, uint8_t batt)
{
    draw_str(20, 0, "-- RAW DATA --");
    char l1[24];
    snprintf(l1, sizeof(l1), "Accel: %.2f %.2f", ax, ay);
    draw_str(0, LINE_H, l1);
    char l2[24];
    snprintf(l2, sizeof(l2), "       %.2f g", az);
    draw_str(0, LINE_H * 2, l2);
    char l3[24];
    snprintf(l3, sizeof(l3), "Gyro:  %.1f %.1f", gx, gy);
    draw_str(0, LINE_H * 3, l3);
    char l4[24];
    snprintf(l4, sizeof(l4), "       %.1f dps", gz);
    draw_str(0, LINE_H * 4, l4);
    char l5[24];
    snprintf(l5, sizeof(l5), "P:%.0fPa T:%.1fC", press_pa, temp);
    draw_str(0, LINE_H * 5, l5);
    char l6[24];
    snprintf(l6, sizeof(l6), "Batt: %d%%", batt);
    draw_str(0, LINE_H * 6, l6);
}

#define COMPASS_CX 64
#define COMPASS_CY 34
#define COMPASS_R  24

static void draw_layout_compass(float heading, int mag_ok)
{
    draw_str(28, 0, "COMPASS");
    fb_draw_circle(COMPASS_CX, COMPASS_CY, COMPASS_R);
    draw_char(COMPASS_CX - 2, COMPASS_CY - COMPASS_R - 9, 'N');
    draw_char(COMPASS_CX + COMPASS_R + 2, COMPASS_CY - 4, 'E');
    draw_char(COMPASS_CX - 2, COMPASS_CY + COMPASS_R + 2, 'S');
    draw_char(COMPASS_CX - COMPASS_R - 9, COMPASS_CY - 4, 'W');
    float rad = heading * (float)M_PI / 180.0f;
    int tip_x  = COMPASS_CX + (int)((COMPASS_R - 3) * sinf(rad));
    int tip_y  = COMPASS_CY - (int)((COMPASS_R - 3) * cosf(rad));
    int tail_x = COMPASS_CX - (int)((COMPASS_R - 3) * 0.4f * sinf(rad));
    int tail_y = COMPASS_CY + (int)((COMPASS_R - 3) * 0.4f * cosf(rad));
    fb_draw_line(tail_x, tail_y, tip_x, tip_y);
    fb_fill_rect(COMPASS_CX - 1, COMPASS_CY - 1, 2, 2, 1);
    char line[24];
    snprintf(line, sizeof(line), "%.0fdeg %s%s", heading, heading_to_cardinal(heading),
             mag_ok ? "" : " (loi cam bien)");
    draw_str(16, OLED_HEIGHT - LINE_H, line);
}

esp_err_t display_driver_init(void)
{
    if (i2c1_bus == NULL) {
        ESP_LOGE(TAG, "I2C1 bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_I2C_ADDR,
        .scl_speed_hz = I2C_OLED_CLK_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(i2c1_bus, &dev_cfg, &s_oled_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF,
        0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF,
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        err = oled_cmd(init_cmds[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SSD1306 cmd 0x%02X failed", init_cmds[i]);
            return err;
        }
    }

    fb_clear();
    fb_flush();
    ESP_LOGI(TAG, "SSD1306 init OK @0x%02X on I2C1(GPIO1/2)", SSD1306_I2C_ADDR);
    return ESP_OK;
}

esp_err_t display_driver_update(system_state_t state, uint8_t battery_pct,
                                 float heading_deg,
                                 float imu_ax, float imu_ay, float imu_az,
                                 float imu_gx, float imu_gy, float imu_gz,
                                 float altitude_m, float temperature_c,
                                 float pressure_pa,
                                 bool gnss_has_fix, bool mag_ok)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_toggle_us >= (int64_t)DISPLAY_SCREEN_INTERVAL_MS * 1000) {
        s_screen_index = (s_screen_index + 1) % DISPLAY_SCREEN_COUNT;
        s_last_toggle_us = now;
    }

    fb_clear();

    /* Nếu đang cấp cứu, ưu tiên hiển thị màn hình SOS chớp nháy */
    if (state == SYS_STATE_SOS) {
        static bool blink = false;
        blink = !blink;
        if (blink) {
            fb_fill_rect(10, 10, OLED_WIDTH - 20, 30, 1);
            /* Do chưa có hàm vẽ text chữ đen nền trắng, vẽ thủ công mảng nhỏ hoặc giữ đơn giản */
        }
        draw_str(35, 10, "!!! S O S !!!");
        draw_str(15, 30, "DANG PHAT LORA...");
        char line[32];
        snprintf(line, sizeof(line), "PIN:%d%% GPS:%s", battery_pct, gnss_has_fix ? "OK" : "WAIT");
        draw_str(0, 50, line);
        return fb_flush();
    }

    switch (s_screen_index) {
        case 0:
            draw_layout_status(state, battery_pct, heading_deg, altitude_m,
                                temperature_c, pressure_pa, gnss_has_fix, mag_ok);
            break;
        case 1:
            draw_layout_raw(imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz,
                             pressure_pa, temperature_c, battery_pct);
            break;
        case 2:
            draw_layout_compass(heading_deg, mag_ok);
            break;
    }

    return fb_flush();
}

bool display_driver_should_update(uint32_t min_interval_ms)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_update_us < (int64_t)min_interval_ms * 1000) {
        return false;
    }
    s_last_update_us = now;
    return true;
}
