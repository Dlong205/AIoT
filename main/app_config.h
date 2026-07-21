#pragma once
/**
 * app_config.h
 * Cấu hình phần cứng prototype:
 *   ESP32-S3 N16R8 | MPU6050 (GY-521) | QMC5883L (GY-271)
 *   GY-NEO7M (GNSS, UART, NMEA) | SX1262 (LoRa, SPI)
 *   OLED SSD1306 0.96" (I2C) | Nút SOS/Cancel + Buzzer | ADC đo pin
 *
 * TODO: chỉnh lại số chân GPIO theo board thực tế của bạn.
 */

/* ---------- I2C0: cảm biến (MPU9250 + QMC5883L + BMP280) ---------- */
#define I2C_BUS_PORT        I2C_NUM_0
#define I2C_SDA_GPIO        8
#define I2C_SCL_GPIO        9
#define I2C_CLK_HZ          100000

#define MPU6050_I2C_ADDR    0x68
#define QMC5883L_I2C_ADDR   0x2C
#define BMP280_I2C_ADDR      0x76

/* ---------- I2C1: OLED SSD1306 riêng ---------- */
#define I2C_OLED_BUS_PORT   I2C_NUM_1
#define I2C_OLED_SDA_GPIO   2
#define I2C_OLED_SCL_GPIO   1
#define I2C_OLED_CLK_HZ     100000
#define SSD1306_I2C_ADDR    0x3C

/* ---------- UART cho GY-NEO7M (GNSS) ---------- */
#define GNSS_UART_PORT      UART_NUM_1
#define GNSS_UART_TX_GPIO   17
#define GNSS_UART_RX_GPIO   18
#define GNSS_UART_BAUD      9600
#define GNSS_PPS_GPIO       -1   /* không dùng chân PPS ở prototype */

/* GY-NEO7M không có chân bật/tắt nguồn riêng như module có EN,
 * nên "tắt GNSS" ở prototype = cắt nguồn qua MOSFET/load-switch điều
 * khiển bằng GPIO này (khuyến nghị thêm mạch ngoài để tiết kiệm pin). */
#define GNSS_POWER_EN_GPIO  3

/* ---------- SPI bus cho SX1262 ---------- */
#define LORA_SPI_HOST       SPI2_HOST
#define LORA_SCK_GPIO       33//
#define LORA_MOSI_GPIO      34//
#define LORA_MISO_GPIO      35//
#define LORA_CS_GPIO        36//
#define LORA_RST_GPIO       5
#define LORA_BUSY_GPIO      6
#define LORA_DIO1_GPIO      7

/* ---------- UART cho AS32 (LoRa module AT-command qua UART) ---------- */
#define AS32_UART_PORT       UART_NUM_2
#define AS32_UART_TX_GPIO    39
#define AS32_UART_RX_GPIO    48
#define AS32_UART_BAUD       9600
#define AS32_M0_GPIO         41
#define AS32_M1_GPIO         40
//#define aux

/* ---------- Nút bấm + buzzer ---------- */
#define BTN_SOS_GPIO        4
#define BTN_CANCEL_GPIO     14   /* ĐÃ SỬA: trước đây = 5, trùng LORA_RST_GPIO */
#define BUZZER_GPIO         21   /* ĐÃ SỬA: trước đây = 6, trùng LORA_BUSY_GPIO */

/* ---------- ADC đo pin (cầu phân áp) ---------- */
#define BATTERY_ADC_UNIT    ADC_UNIT_1
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_4   /* ĐÃ SỬA: ADC_CHANNEL_4 = GPIO 5 (trước đây là CH3 = GPIO 4, trùng với nút SOS!) */
#define BATTERY_DIVIDER_RATIO  2.0f         /* R1=R2 -> chia đôi điện áp */

/* ---------- Hằng số hệ thống ---------- */
#define SOS_CONFIRM_TIMEOUT_MS      8000    /* thời gian chờ bấm Cancel */
#define HEARTBEAT_INTERVAL_NORMAL_MS   60000
#define HEARTBEAT_INTERVAL_LOWBAT_MS  180000
#define BREADCRUMB_MIN_DISTANCE_M   15.0f
#define BREADCRUMB_MIN_HEADING_DEG  35.0f
#define GNSS_FIX_BUFFER_COUNT        3      /* số fix liên tiếp trước khi tin P0 */

/* ---------- SPI cho SD Card (lưu dữ liệu train AI) ---------- */
#define SD_SPI_HOST         SPI3_HOST
#define SD_CS_GPIO          10
#define SD_MOSI_GPIO        12
#define SD_MISO_GPIO        13
#define SD_SCLK_GPIO        11
#define SD_MOUNT_POINT      "/sdcard"
