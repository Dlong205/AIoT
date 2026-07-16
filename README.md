# esp32_rescue_node — Khung sườn code ESP-IDF cho node thiết bị đeo

Đây là **cấu trúc project + khung sườn (skeleton)**, chưa phải bản chạy hoàn
chỉnh. Các hàm driver phần cứng (I2C/UART/SPI cụ thể) được đánh dấu `TODO` —
đó là phần cần viết tiếp theo từng linh kiện thật của bạn.

## Build

```bash
. $HOME/esp/esp-idf/export.sh   # nếu chưa export môi trường ESP-IDF
idf.py set-target esp32s3
idf.py menuconfig               # kiểm tra lại sdkconfig.defaults đã áp đúng chưa
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Cấu trúc thư mục

```
esp32_rescue_node/
├── main/
│   ├── main.c          # app_main: init bus, tạo 6 FreeRTOS task
│   └── app_config.h    # toàn bộ chân GPIO, hằng số hệ thống
└── components/
    ├── common/              # rescue_types.h - struct/enum dùng chung
    ├── imu_driver/           # MPU6050 (GY-521), I2C
    ├── mag_driver/           # QMC5883L (GY-271), I2C
    ├── gnss_driver/          # GY-NEO7M, UART + NMEA parser
    ├── lora_driver/          # SX1262, SPI
    ├── display_driver/       # OLED SSD1306 0.96", I2C
    ├── activity_classifier/  # threshold-based, chưa cần TFLite Micro
    ├── pdr/                  # Pedestrian Dead Reckoning + breadcrumb
    ├── state_machine/        # FSM 6 trạng thái (Bảng 5)
    ├── battery_monitor/      # ADC + cầu phân áp
    ├── power_mgmt/           # Deep Sleep, GNSS power toggle
    └── mesh_protocol/        # định dạng gói SOS pack/unpack
```

## Bản đồ luồng dữ liệu (khớp sơ đồ đã thống nhất)

```
MPU6050 (I2C) ──► imu_task ──► activity_classifier (threshold)
                                      │
                                      ▼ event: FALL_SUSPECTED / STEP_DETECTED
                              state_machine_task ◄── nút SOS/Cancel (GPIO)
                                      │
                    ┌─────────────────┼─────────────────┐
                    ▼                 ▼                  ▼
            pdr_task (trong    gnss_task (UART,     lora_task (SPI,
            imu_task, dùng      GY-NEO7M, chỉ         SX1262, chỉ gửi
            QMC5883L heading)   chạy khi cần P0)      khi có SOS/heartbeat)
                    │                 │                  │
                    └────────► display_task (I2C OLED) ◄─┘
```
*(Ghi chú: trong khung sườn này, bước PDR được gọi trực tiếp từ `imu_task`
thay vì tách task riêng, để tránh đồng bộ hoá phức tạp cho bản đầu tiên. Có
thể tách ra `pdr_task` + queue riêng nếu cần.)*

## Việc cần làm tiếp theo (theo thứ tự ưu tiên đề xuất)

1. **imu_driver.c / mag_driver.c**: viết I2C read/write thật cho MPU6050 và
   QMC5883L (datasheet có sẵn thanh ghi trong comment).
2. **gnss_driver.c / nmea_parser.c**: parse câu `$GxGGA` thật từ GY-NEO7M,
   test bằng cách `idf.py monitor` xem log NMEA thô trước khi parse.
3. **activity_classifier.c**: thu thập vài chục mẫu ngã thật/giả (đi bộ,
   chạy, ngồi xuống nhanh, làm rơi thiết bị) để tinh chỉnh ngưỡng
   `TH_FREEFALL_MAG_MAX` / `TH_IMPACT_MAG_MIN`.
4. **lora_driver.c**: driver SX1262 SPI — có thể tham khảo lib RadioLib
   (port sang ESP-IDF) thay vì viết lại từ đầu nếu muốn tiết kiệm thời gian.
5. **state_machine.c**: bổ sung timer cho `SYS_STATE_FALL_SUSPECTED`
   (đếm ngược `SOS_CONFIRM_TIMEOUT_MS`) bằng `esp_timer`.
6. **display_driver.c**: chọn 1 lib SSD1306 nhẹ (hoặc viết tối giản vì chỉ
   cần text + 1 mũi tên) rồi nối vào `display_driver_draw_status`.

## Lưu ý đúng như đã phân tích trong tài liệu dự án

- MPU6050 không có MLC → `imu_task` phải chạy liên tục để classify, không
  thể để IMU tự đánh thức MCU khi có sự kiện ngã (chỉ dùng wake-on-motion
  để thoát Deep Sleep, xem `imu_driver_enable_wake_on_motion`).
- GY-NEO7M không có chân EN riêng như một số module GNSS khác — việc "tắt
  GNSS" trong `gnss_driver_power()` giả định có mạch load-switch ngoài
  (MOSFET) điều khiển qua `GNSS_POWER_EN_GPIO`. Nếu prototype chưa có mạch
  này, hàm sẽ không thực sự cắt dòng tĩnh của module.
- `battery_monitor` dùng đường cong tuyến tính hoá đơn giản — %pin chỉ mang
  tính tương đối, không dùng để tính runtime chính xác.
=======
# AIoT
>>>>>>> 68513e8d84a08d3d8cc283fe3c40a7d435ad657c
