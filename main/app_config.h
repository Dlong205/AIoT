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

/* ---------- I2C bus dùng chung cho MPU6050 + QMC5883L + OLED ---------- */
#define I2C_BUS_PORT        I2C_NUM_0
#define I2C_SDA_GPIO        8
#define I2C_SCL_GPIO        9
#define I2C_CLK_HZ          400000

#define MPU6050_I2C_ADDR    0x68
#define QMC5883L_I2C_ADDR   0x0D
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
#define GNSS_POWER_EN_GPIO  4

/* ---------- SPI bus cho SX1262 ---------- */
#define LORA_SPI_HOST       SPI2_HOST
#define LORA_SCK_GPIO       12
#define LORA_MOSI_GPIO      11
#define LORA_MISO_GPIO      13
#define LORA_CS_GPIO        10
#define LORA_RST_GPIO       5
#define LORA_BUSY_GPIO      6
#define LORA_DIO1_GPIO      7

/* ---------- Nút bấm + buzzer ---------- */
#define BTN_SOS_GPIO        14
#define BTN_CANCEL_GPIO     15
#define BUZZER_GPIO         16

/* ---------- ADC đo pin (cầu phân áp) ---------- */
#define BATTERY_ADC_UNIT    ADC_UNIT_1
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_3   /* GPIO tương ứng tùy board */
#define BATTERY_DIVIDER_RATIO  2.0f         /* R1=R2 -> chia đôi điện áp */

/* ---------- Hằng số hệ thống ---------- */
#define SOS_CONFIRM_TIMEOUT_MS      8000    /* thời gian chờ bấm Cancel */
#define HEARTBEAT_INTERVAL_NORMAL_MS   60000
#define HEARTBEAT_INTERVAL_LOWBAT_MS  180000
#define BREADCRUMB_MIN_DISTANCE_M   15.0f
#define BREADCRUMB_MIN_HEADING_DEG  35.0f
#define GNSS_FIX_BUFFER_COUNT        3      /* số fix liên tiếp trước khi tin P0 */
