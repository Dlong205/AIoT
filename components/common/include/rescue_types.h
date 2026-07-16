#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ================= 6 trạng thái hệ thống (Bảng 5) ================= */
typedef enum {
    SYS_STATE_IDLE = 0,      /* đứng yên, IMU sleep-wake */
    SYS_STATE_WALKING,       /* đang di chuyển bình thường */
    SYS_STATE_FALL_SUSPECTED,/* nghi ngờ ngã, đang đếm ngược xác nhận */
    SYS_STATE_SOS,           /* đã phát gói SOS, chờ cứu hộ */
    SYS_STATE_RETURN,        /* đang dẫn đường quay lại theo breadcrumb */
    SYS_STATE_LOW_POWER      /* pin yếu, giảm tần suất mọi hoạt động */
} system_state_t;

/* ================= Sự kiện nội bộ giữa các task ================= */
typedef enum {
    EVT_STEP_DETECTED = 0,
    EVT_FALL_SUSPECTED,
    EVT_FALL_CONFIRMED,      /* hết timeout mà không Cancel */
    EVT_BTN_SOS_PRESSED,
    EVT_BTN_CANCEL_PRESSED,
    EVT_GNSS_FIX_ACQUIRED,
    EVT_GNSS_FIX_LOST,
    EVT_BATTERY_LOW,
    EVT_RETURN_REQUESTED,
    EVT_RETURN_ARRIVED,
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    int64_t timestamp_us;
} app_event_t;

/* ================= Vị trí (tuyệt đối hoặc tương đối) ================= */
typedef struct {
    bool     is_absolute;    /* true = có tọa độ GNSS thật, false = PDR */
    double   lat;            /* dùng khi is_absolute == true */
    double   lon;
    float    rel_x_m;        /* dùng khi is_absolute == false (so với P0) */
    float    rel_y_m;
    float    accuracy_m;     /* ước lượng sai số - KHÔNG tuyên bố tuyệt đối */
    int64_t  timestamp_us;
} position_t;

/* Breadcrumb: điểm mốc lưu lại trong hành trình để dẫn đường quay lại */
typedef struct {
    float rel_x_m;
    float rel_y_m;
    float heading_deg;
} breadcrumb_t;

/* ================= Mẫu cảm biến IMU thô ================= */
typedef struct {
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* deg/s */
    int64_t timestamp_us;
} imu_sample_t;

typedef enum {
    ACTIVITY_STILL = 0,
    ACTIVITY_WALKING,
    ACTIVITY_RUNNING,
    ACTIVITY_STUMBLE,
    ACTIVITY_FALL,
    ACTIVITY_DEVICE_DROPPED,
} activity_class_t;

/* ================= Gói tin SOS phát qua LoRa mesh ================= */
typedef struct __attribute__((packed)) {
    uint32_t source_id;
    uint32_t message_id;
    uint8_t  packet_type;     /* 0=SOS, 1=HEARTBEAT, 2=ACK ... xem mesh_protocol.h */
    uint8_t  confidence;      /* 0-100, độ tin cậy của activity_classifier */
    double   lat;
    double   lon;
    float    rel_x_m;
    float    rel_y_m;
    bool     position_is_absolute;
    uint8_t  battery_pct;
    uint8_t  hop_count;
    uint8_t  hop_limit;
} sos_packet_t;
