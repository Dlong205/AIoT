#include "display_driver.h"
#include "app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "display_driver";
static int64_t s_last_update_us = 0;
static int64_t s_last_toggle_us = 0;
static int s_debug_mode = 0;

#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_PAGES  (OLED_HEIGHT / 8)
#define FB_SIZE     (OLED_WIDTH * OLED_PAGES)
#define FONT_W      5
#define FONT_H      7
#define CHAR_W      (FONT_W + 1)
#define LINE_H      (FONT_H + 1)
#define MAX_LINES   (OLED_HEIGHT / LINE_H)

static uint8_t s_fb[FB_SIZE];

/* ==================== I2C helpers ==================== */

static esp_err_t oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_write_to_device(I2C_OLED_BUS_PORT, SSD1306_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t oled_data_bulk(const uint8_t *data, size_t len)
{
    /* Gửi 0x40 + data, chunk tối đa 128 byte */
    size_t sent = 0;
    while (sent < len) {
        size_t chunk = len - sent;
        if (chunk > 127) chunk = 127;
        uint8_t buf[128];
        buf[0] = 0x40;
        memcpy(buf + 1, data + sent, chunk);
        esp_err_t err = i2c_master_write_to_device(I2C_OLED_BUS_PORT, SSD1306_I2C_ADDR, buf, chunk + 1, pdMS_TO_TICKS(500));
        if (err != ESP_OK) return err;
        sent += chunk;
    }
    return ESP_OK;
}

/* ==================== Framebuffer primitives ==================== */

static void fb_clear(void)
{
    memset(s_fb, 0, FB_SIZE);
}

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

/* Thử 0 (SSD1306 chuẩn), nếu lệch hình đổi thành 2 (SH1106 clone) */
#define OLED_COL_OFFSET  0

static esp_err_t fb_flush(void)
{
    /* Page Addressing Mode (tương thích SSD1306 + SH1106) */
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

/* ==================== Font 5x7 ==================== */
/* Standard 5x7 font, index: (c - 0x20) * 5, c in [0x20..0x7E] */
static const uint8_t s_font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, /* space */
    0x00, 0x00, 0x5F, 0x00, 0x00, /* ! */
    0x00, 0x07, 0x00, 0x07, 0x00, /* " */
    0x14, 0x7F, 0x14, 0x7F, 0x14, /* # */
    0x24, 0x2A, 0x7F, 0x2A, 0x12, /* $ */
    0x23, 0x13, 0x08, 0x64, 0x62, /* % */
    0x36, 0x49, 0x55, 0x22, 0x50, /* & */
    0x00, 0x05, 0x03, 0x00, 0x00, /* ' */
    0x00, 0x1C, 0x22, 0x41, 0x00, /* ( */
    0x00, 0x41, 0x22, 0x1C, 0x00, /* ) */
    0x08, 0x2A, 0x1C, 0x2A, 0x08, /* * */
    0x08, 0x08, 0x3E, 0x08, 0x08, /* + */
    0x00, 0x50, 0x30, 0x00, 0x00, /* , */
    0x08, 0x08, 0x08, 0x08, 0x08, /* - */
    0x00, 0x60, 0x60, 0x00, 0x00, /* . */
    0x20, 0x10, 0x08, 0x04, 0x02, /* / */
    0x3E, 0x51, 0x49, 0x45, 0x3E, /* 0 */
    0x00, 0x42, 0x7F, 0x40, 0x00, /* 1 */
    0x42, 0x61, 0x51, 0x49, 0x46, /* 2 */
    0x21, 0x41, 0x45, 0x4B, 0x31, /* 3 */
    0x18, 0x14, 0x12, 0x7F, 0x10, /* 4 */
    0x27, 0x45, 0x45, 0x45, 0x39, /* 5 */
    0x3C, 0x4A, 0x49, 0x49, 0x30, /* 6 */
    0x01, 0x71, 0x09, 0x05, 0x03, /* 7 */
    0x36, 0x49, 0x49, 0x49, 0x36, /* 8 */
    0x06, 0x49, 0x49, 0x29, 0x1E, /* 9 */
    0x00, 0x36, 0x36, 0x00, 0x00, /* : */
    0x00, 0x56, 0x36, 0x00, 0x00, /* ; */
    0x00, 0x08, 0x14, 0x22, 0x41, /* < */
    0x14, 0x14, 0x14, 0x14, 0x14, /* = */
    0x41, 0x22, 0x14, 0x08, 0x00, /* > */
    0x02, 0x01, 0x51, 0x09, 0x06, /* ? */
    0x32, 0x49, 0x79, 0x41, 0x3E, /* @ */
    0x7E, 0x11, 0x11, 0x11, 0x7E, /* A */
    0x7F, 0x49, 0x49, 0x49, 0x36, /* B */
    0x3E, 0x41, 0x41, 0x41, 0x22, /* C */
    0x7F, 0x41, 0x41, 0x22, 0x1C, /* D */
    0x7F, 0x49, 0x49, 0x49, 0x41, /* E */
    0x7F, 0x09, 0x09, 0x01, 0x01, /* F */
    0x3E, 0x41, 0x41, 0x51, 0x32, /* G */
    0x7F, 0x08, 0x08, 0x08, 0x7F, /* H */
    0x00, 0x41, 0x7F, 0x41, 0x00, /* I */
    0x20, 0x40, 0x41, 0x3F, 0x01, /* J */
    0x7F, 0x08, 0x14, 0x22, 0x41, /* K */
    0x7F, 0x40, 0x40, 0x40, 0x40, /* L */
    0x7F, 0x02, 0x04, 0x02, 0x7F, /* M */
    0x7F, 0x04, 0x08, 0x10, 0x7F, /* N */
    0x3E, 0x41, 0x41, 0x41, 0x3E, /* O */
    0x7F, 0x09, 0x09, 0x09, 0x06, /* P */
    0x3E, 0x41, 0x51, 0x21, 0x5E, /* Q */
    0x7F, 0x09, 0x19, 0x29, 0x46, /* R */
    0x46, 0x49, 0x49, 0x49, 0x31, /* S */
    0x01, 0x01, 0x7F, 0x01, 0x01, /* T */
    0x3F, 0x40, 0x40, 0x40, 0x3F, /* U */
    0x1F, 0x20, 0x40, 0x20, 0x1F, /* V */
    0x7F, 0x20, 0x18, 0x20, 0x7F, /* W */
    0x63, 0x14, 0x08, 0x14, 0x63, /* X */
    0x03, 0x04, 0x78, 0x04, 0x03, /* Y */
    0x61, 0x51, 0x49, 0x45, 0x43, /* Z */
    0x00, 0x00, 0x7F, 0x41, 0x41, /* [ */
    0x02, 0x04, 0x08, 0x10, 0x20, /* \ */
    0x41, 0x41, 0x7F, 0x00, 0x00, /* ] */
    0x04, 0x02, 0x01, 0x02, 0x04, /* ^ */
    0x40, 0x40, 0x40, 0x40, 0x40, /* _ */
    0x00, 0x01, 0x02, 0x04, 0x00, /* ` */
    0x20, 0x54, 0x54, 0x54, 0x78, /* a */
    0x7F, 0x48, 0x44, 0x44, 0x38, /* b */
    0x38, 0x44, 0x44, 0x44, 0x20, /* c */
    0x38, 0x44, 0x44, 0x48, 0x7F, /* d */
    0x38, 0x54, 0x54, 0x54, 0x18, /* e */
    0x08, 0x7E, 0x09, 0x01, 0x02, /* f */
    0x08, 0x14, 0x54, 0x54, 0x3C, /* g */
    0x7F, 0x08, 0x04, 0x04, 0x78, /* h */
    0x00, 0x44, 0x7D, 0x40, 0x00, /* i */
    0x20, 0x40, 0x44, 0x3D, 0x00, /* j */
    0x00, 0x7F, 0x10, 0x28, 0x44, /* k */
    0x00, 0x41, 0x7F, 0x40, 0x00, /* l */
    0x7C, 0x04, 0x18, 0x04, 0x78, /* m */
    0x7C, 0x08, 0x04, 0x04, 0x78, /* n */
    0x38, 0x44, 0x44, 0x44, 0x38, /* o */
    0x7C, 0x14, 0x14, 0x14, 0x08, /* p */
    0x08, 0x14, 0x14, 0x18, 0x7C, /* q */
    0x7C, 0x08, 0x04, 0x04, 0x08, /* r */
    0x48, 0x54, 0x54, 0x54, 0x20, /* s */
    0x04, 0x3F, 0x44, 0x40, 0x20, /* t */
    0x3C, 0x40, 0x40, 0x20, 0x7C, /* u */
    0x1C, 0x20, 0x40, 0x20, 0x1C, /* v */
    0x3C, 0x40, 0x30, 0x40, 0x3C, /* w */
    0x44, 0x28, 0x10, 0x28, 0x44, /* x */
    0x0C, 0x50, 0x50, 0x50, 0x3C, /* y */
    0x44, 0x64, 0x54, 0x4C, 0x44, /* z */
    0x00, 0x08, 0x36, 0x41, 0x00, /* { */
    0x00, 0x00, 0x7F, 0x00, 0x00, /* | */
    0x00, 0x41, 0x36, 0x08, 0x00, /* } */
    0x08, 0x08, 0x2A, 0x1C, 0x08, /* -> */
    0x08, 0x1C, 0x2A, 0x08, 0x08, /* <- */
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

/* ==================== Icon drawing ==================== */

static void draw_icon_battery(int x, int y, int pct)
{
    /* Battery outline: 16x8 */
    int w = 14, h = 6;
    fb_fill_rect(x, y, w, h, 1);
    fb_fill_rect(x, y, w, h, 0);              /* clear inside */
    fb_fill_rect(x + w, y + 1, 2, h - 2, 1);  /* terminal nub */
    /* Fill level */
    int bars = (pct + 10) / 20;  /* 0..5 */
    if (bars > 5) bars = 5;
    int fill_w = (w - 2) * bars / 5;
    if (bars > 0)
        fb_fill_rect(x + 1, y + 1, fill_w, h - 2, 1);
    /* Draw outline again to keep borders visible */
    fb_fill_rect(x, y, w, 1, 1);
    fb_fill_rect(x, y + h - 1, w, 1, 1);
    fb_fill_rect(x, y + 1, 1, h - 2, 1);
    fb_fill_rect(x + w - 1, y + 1, 1, h - 2, 1);
}

static void draw_icon_state(int x, int y, system_state_t state)
{
    /* Simple 8x8 icon per state using a box + text approach */
    fb_fill_rect(x, y, 9, 9, 1);
    fb_fill_rect(x + 1, y + 1, 7, 7, 0);
    char c = '?';
    switch (state) {
        case SYS_STATE_IDLE:     c = 'Z'; break;
        case SYS_STATE_WALKING:  c = 'W'; break;
        case SYS_STATE_FALL_SUSPECTED: c = '!'; break;
        case SYS_STATE_SOS:      c = 'S'; break;
        case SYS_STATE_RETURN:   c = 'R'; break;
        case SYS_STATE_LOW_POWER:c = 'L'; break;
        default:                 c = '?'; break;
    }
    draw_char(x + 2, y + 1, c);
}

static void draw_icon_gnss(int x, int y, int has_fix)
{
    /* Simple circle-like crosshair */
    fb_fill_rect(x + 3, y, 2, 8, 1);
    fb_fill_rect(x, y + 3, 8, 2, 1);
    if (!has_fix) {
        /* Cross out */
        fb_fill_rect(x, y, 1, 1, 1);
        fb_fill_rect(x + 7, y, 1, 1, 1);
        fb_fill_rect(x, y + 7, 1, 1, 1);
        fb_fill_rect(x + 7, y + 7, 1, 1, 1);
    }
}

/* ==================== Layouts ==================== */

static void draw_layout_normal(system_state_t state, uint8_t batt,
                                float heading, float alt, float temp, float press,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                int gnss_fix, int mag_ok)
{
    /* Line 0: icon + state + battery */
    draw_icon_state(0, 0, state);
    const char *st = "";
    switch (state) {
        case SYS_STATE_IDLE:     st = "IDLE"; break;
        case SYS_STATE_WALKING:  st = "WALKING"; break;
        case SYS_STATE_FALL_SUSPECTED: st = "FALL?"; break;
        case SYS_STATE_SOS:      st = "SOS!"; break;
        case SYS_STATE_RETURN:   st = "RETURN"; break;
        case SYS_STATE_LOW_POWER:st = "LOW PWR"; break;
    }
    draw_str(12, 0, st);
    char batt_str[8];
    snprintf(batt_str, sizeof(batt_str), "%d%%", batt);
    int batt_x = OLED_WIDTH - CHAR_W * strlen(batt_str);
    draw_icon_battery(batt_x - 18, 0, batt);
    draw_str(batt_x, 0, batt_str);

    /* Line 1: heading + alt */
    char line1[24];
    snprintf(line1, sizeof(line1), "H:%.1f ALT:%.0fm", heading, alt);
    draw_str(0, LINE_H, line1);

    /* Line 2: accel */
    char line2[24];
    snprintf(line2, sizeof(line2), "A %.2f %.2f %.2f", ax, ay, az);
    draw_str(0, LINE_H * 2, line2);

    /* Line 3: gyro */
    char line3[24];
    snprintf(line3, sizeof(line3), "G %.1f %.1f %.1f", gx, gy, gz);
    draw_str(0, LINE_H * 3, line3);

    /* Line 4: alt + temp + press (hPa) */
    char line4[24];
    snprintf(line4, sizeof(line4), "%.0fm %.1fC %.0fhPa", alt, temp, press / 100.0f);
    draw_str(0, LINE_H * 4, line4);

    /* Line 5: GNSS + MAG status */
    draw_icon_gnss(0, LINE_H * 5, gnss_fix);
    draw_str(10, LINE_H * 5, gnss_fix ? "GNSS ok" : "GNSS no");
    draw_str(OLED_WIDTH - 30, LINE_H * 5, mag_ok ? "MAG ok" : "MAG no");
}

static void draw_layout_debug(system_state_t state, uint8_t batt,
                               float heading, float alt, float temp, float press,
                               float ax, float ay, float az,
                               float gx, float gy, float gz,
                               int gnss_fix, int mag_ok)
{
    (void)state;
    draw_str(20, 0, "=== DEBUG ===");

    char l1[24];
    snprintf(l1, sizeof(l1), "A %.3f %.3f", ax, ay);
    draw_str(0, LINE_H, l1);

    char l2[24];
    snprintf(l2, sizeof(l2), "Z%.3f T:%.1fC", az, temp);
    draw_str(0, LINE_H * 2, l2);

    char l3[24];
    snprintf(l3, sizeof(l3), "G %.1f %.1f %.1f", gx, gy, gz);
    draw_str(0, LINE_H * 3, l3);

    char l4[24];
    snprintf(l4, sizeof(l4), "H:%.1f A:%.1f %.0fhPa", heading, alt, press / 100.0f);
    draw_str(0, LINE_H * 4, l4);

    char l5[24];
    snprintf(l5, sizeof(l5), "BAT:%d%% %s %s", batt,
             gnss_fix ? "FIX" : "NO",
             mag_ok ? "MAG" : "NOM");
    draw_str(0, LINE_H * 5, l5);
}

/* ==================== Init ==================== */

esp_err_t display_driver_init(void)
{
    esp_err_t err;
    /* Init I2C1 bus for OLED */
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_OLED_SDA_GPIO,
        .scl_io_num = I2C_OLED_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_OLED_CLK_HZ,
    };
    err = i2c_param_config(I2C_OLED_BUS_PORT, &cfg);
    if (err != ESP_OK) return err;
    err = i2c_driver_install(I2C_OLED_BUS_PORT, cfg.mode, 0, 0, 0);
    if (err != ESP_OK) return err;

    /* SSD1306 init sequence */
    vTaskDelay(pdMS_TO_TICKS(100));
    const uint8_t init_cmds[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* oscillator freq */
        0xA8, 0x3F, /* MUX ratio */
        0xD3, 0x00, /* display offset */
        0x40,       /* start line 0 */
        0x8D, 0x14, /* charge pump on */
        0xA1,       /* segment remap (col 127 = SEG0) */
        0xC8,       /* COM scan remapped */
        0xDA, 0x12, /* COM pins */
        0x81, 0xCF, /* contrast max */
        0xD9, 0xF1, /* pre-charge */
        0xDB, 0x40, /* VCOMH */
        0xA4,       /* display on resume */
        0xA6,       /* normal display (not inverted) */
        0x2E,       /* deactivate scroll */
        0xAF,       /* display on */
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

/* ==================== Update ==================== */

esp_err_t display_driver_update(system_state_t state, uint8_t battery_pct,
                                 float heading_deg,
                                 float imu_ax, float imu_ay, float imu_az,
                                 float imu_gx, float imu_gy, float imu_gz,
                                 float altitude_m, float temperature_c,
                                 float pressure_pa,
                                 bool gnss_has_fix, bool mag_ok)
{
    /* Auto-toggle between NORMAL and DEBUG every 5s */
    int64_t now = esp_timer_get_time();
    if (now - s_last_toggle_us >= 5000000) {
        s_debug_mode = !s_debug_mode;
        s_last_toggle_us = now;
    }

    fb_clear();

    if (s_debug_mode) {
        draw_layout_debug(state, battery_pct, heading_deg, altitude_m,
                          temperature_c, pressure_pa,
                          imu_ax, imu_ay, imu_az,
                          imu_gx, imu_gy, imu_gz,
                          gnss_has_fix, mag_ok);
    } else {
        draw_layout_normal(state, battery_pct, heading_deg, altitude_m,
                           temperature_c, pressure_pa,
                           imu_ax, imu_ay, imu_az,
                           imu_gx, imu_gy, imu_gz,
                           gnss_has_fix, mag_ok);
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