#pragma once
#include "esp_err.h"
#include "rescue_types.h"

/* ==================== MPU9250 (dùng như MPU6500, 6 trục accel+gyro) ====
 * QMC5883L (magnetometer) được xử lý riêng bởi mag_driver, không dùng
 * AK8963 nội bộ của MPU9250 -> không bật I2C bypass/master ở đây.
 * ======================================================================== */

/* --- Register map (MPU6500/MPU9250, khác MPU6050 chủ yếu ở WHO_AM_I) --- */
#define MPU9250_REG_WHO_AM_I        0x75
#define MPU9250_WHO_AM_I_VAL        0x71   /* MPU9250/MPU6500 trả về 0x71 */

#define MPU9250_REG_PWR_MGMT_1      0x6B
#define MPU9250_REG_PWR_MGMT_2      0x6C
#define MPU9250_REG_SMPLRT_DIV      0x19
#define MPU9250_REG_CONFIG          0x1A
#define MPU9250_REG_GYRO_CONFIG     0x1B
#define MPU9250_REG_ACCEL_CONFIG    0x1C
#define MPU9250_REG_ACCEL_CONFIG2   0x1D
#define MPU9250_REG_ACCEL_XOUT_H    0x3B

#define MPU9250_REG_INT_PIN_CFG     0x37
#define MPU9250_REG_INT_ENABLE      0x38
#define MPU9250_INT_ENABLE_WOM_BIT  0x40   /* bit6 = WOM_EN */

/* Wake-on-motion (MPU9250 dùng cơ chế Accel Intelligence, khác MPU6050) */
#define MPU9250_REG_WOM_THR         0x1F   /* đơn vị: ~4mg / LSB */
#define MPU9250_REG_ACCEL_INTEL_CTRL 0x69
#define MPU9250_ACCEL_INTEL_EN_BIT   0x80
#define MPU9250_ACCEL_INTEL_MODE_BIT 0x40

/* --- Full-scale range đang dùng: ±8g / ±2000dps --- */
#define MPU9250_GYRO_FS_2000DPS     0x18   /* GYRO_CONFIG[4:3] = 11 */
#define MPU9250_ACCEL_FS_8G         0x10   /* ACCEL_CONFIG[4:3] = 10 */
#define MPU9250_ACCEL_SENSITIVITY_LSB_PER_G   4096.0f
#define MPU9250_GYRO_SENSITIVITY_LSB_PER_DPS  16.4f

/* Khởi tạo MPU9250 trên I2C bus đã init sẵn (dùng chung với mag/OLED).
 * Kiểm tra WHO_AM_I, wake chip, cấu hình DLPF + full-scale ±8g/±2000dps,
 * sample rate 50Hz. */
esp_err_t imu_driver_init(void);

/* Đọc 1 mẫu accel+gyro (accel: g, gyro: dps). Blocking, dùng trong
 * imu_task theo chu kỳ ~20ms (50Hz). */
esp_err_t imu_driver_read(imu_sample_t *out_sample);

/* Thu thập N mẫu tĩnh (board phải đứng yên) để tính bias gyro/accel,
 * lưu lại và trừ vào các lần đọc sau. Gọi 1 lần lúc khởi động, sau
 * imu_driver_init(). Mất khoảng vài trăm ms tuỳ num_samples. */
esp_err_t imu_driver_calibrate(uint16_t num_samples);

/* Cấu hình ngắt wake-on-motion. threshold_mg tính theo mg, được quy đổi
 * sang đơn vị thanh ghi WOM_THR (~4mg/LSB) bên trong hàm. */
esp_err_t imu_driver_enable_wake_on_motion(uint8_t threshold_mg);