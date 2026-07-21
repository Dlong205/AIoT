/*
 * ═══════════════════════════════════════════════════════════════════
 *  SOS RECEIVER — ESP32-S3 + AS32 (LoRa UART)
 *  Arduino IDE
 * ═══════════════════════════════════════════════════════════════════
 *
 *  Nhận gói SOS từ rescue_node qua module AS32, hiển thị:
 *    - Serial Monitor (luôn có)
 *    - OLED SSD1306 0.96" I2C (tuỳ chọn, bỏ comment #define USE_OLED)
 *    - Buzzer cảnh báo (tuỳ chọn)
 *
 *  Phần cứng kết nối:
 *    AS32 TX  → ESP32-S3 RX (GPIO 18)
 *    AS32 RX  → ESP32-S3 TX (GPIO 17)
 *    AS32 M0  → GND (normal mode)
 *    AS32 M1  → GND (normal mode)
 *    AS32 VCC → 5V / 3.3V (tuỳ module)
 *    AS32 GND → GND
 *
 *  Tuỳ chọn OLED:
 *    OLED SDA → GPIO 8
 *    OLED SCL → GPIO 9
 *
 *  Tuỳ chọn Buzzer:
 *    BUZZER   → GPIO 21
 *
 *  Board: "ESP32S3 Dev Module" trong Arduino IDE
 *  Cài thư viện (nếu dùng OLED):
 *    - Adafruit SSD1306
 *    - Adafruit GFX Library
 * ═══════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>

/* ─── Tuỳ chọn: bỏ comment để bật ─── */
// #define USE_OLED          /* Bật nếu có OLED SSD1306 */
// #define USE_BUZZER        /* Bật nếu có buzzer */

/* ─── Cấu hình chân GPIO ─── */
#define AS32_RX_PIN    18    /* ESP32 RX ← AS32 TX */
#define AS32_TX_PIN    17    /* ESP32 TX → AS32 RX */
#define AS32_BAUD      9600

#ifdef USE_OLED
  #define OLED_SDA     8
  #define OLED_SCL     9
  #define OLED_ADDR    0x3C
  #define SCREEN_W     128
  #define SCREEN_H     64
#endif

#ifdef USE_BUZZER
  #define BUZZER_PIN   21
#endif

/* ═══════════════════════════════════════════════════════════════════
 *  Cấu trúc gói tin SOS — PHẢI GIỐNG HỆT bên sender (rescue_types.h)
 * ═══════════════════════════════════════════════════════════════════ */

#define PKT_TYPE_SOS              0
#define PKT_TYPE_HEARTBEAT        1
#define PKT_TYPE_ACK              2
#define PKT_TYPE_REPEATER_FORWARD 3

typedef struct __attribute__((packed)) {
    uint32_t source_id;
    uint32_t message_id;
    uint8_t  packet_type;       /* 0=SOS, 1=HEARTBEAT, 2=ACK, 3=FORWARD */
    uint8_t  confidence;        /* 0–100 */
    double   lat;               /* vĩ độ (GNSS) */
    double   lon;               /* kinh độ (GNSS) */
    float    rel_x_m;           /* vị trí tương đối PDR (m) */
    float    rel_y_m;
    bool     position_is_absolute;  /* true = có GPS, false = chỉ PDR */
    uint8_t  battery_pct;       /* % pin */
    uint8_t  hop_count;         /* số hop đã chuyển tiếp */
    uint8_t  hop_limit;         /* giới hạn hop */
} sos_packet_t;

/* ═══════════════════════════════════════════════════════════════════
 *  OLED (tuỳ chọn)
 * ═══════════════════════════════════════════════════════════════════ */

#ifdef USE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 oled_display(SCREEN_W, SCREEN_H, &Wire, -1);
  bool oled_ok = false;
  void update_oled(const sos_packet_t *pkt);
  void oled_standby();
#endif

/* ─── Biến toàn cục ─── */
static sos_packet_t last_pkt;
static bool has_received = false;
static uint32_t last_rx_time = 0;
static uint32_t total_rx_count = 0;

/* ═══════════════════════════════════════════════════════════════════
 *  Hàm tiện ích
 * ═══════════════════════════════════════════════════════════════════ */

const char* pkt_type_name(uint8_t type) {
    switch (type) {
        case PKT_TYPE_SOS:              return "SOS";
        case PKT_TYPE_HEARTBEAT:        return "HEARTBEAT";
        case PKT_TYPE_ACK:              return "ACK";
        case PKT_TYPE_REPEATER_FORWARD: return "FORWARD";
        default:                        return "UNKNOWN";
    }
}

void buzzer_alert(int beeps) {
#ifdef USE_BUZZER
    for (int i = 0; i < beeps; i++) {
        tone(BUZZER_PIN, 2700, 200);
        delay(300);
    }
#endif
    (void)beeps;
}

void print_separator() {
    Serial.println("======================================================");
}

/* ═══════════════════════════════════════════════════════════════════
 *  Xử lý gói tin nhận được
 * ═══════════════════════════════════════════════════════════════════ */

void handle_packet(const sos_packet_t *pkt) {
    total_rx_count++;
    last_rx_time = millis();
    last_pkt = *pkt;
    has_received = true;

    /* ─── In ra Serial Monitor ─── */
    print_separator();
    Serial.printf(">>> NHAN GOI %s! (lan thu %lu)\n",
                  pkt_type_name(pkt->packet_type), total_rx_count);
    print_separator();

    Serial.printf("  Source ID   : 0x%08lX\n", (unsigned long)pkt->source_id);
    Serial.printf("  Message ID  : %lu\n",     (unsigned long)pkt->message_id);
    Serial.printf("  Loai goi    : %s (%d)\n", pkt_type_name(pkt->packet_type), pkt->packet_type);
    Serial.printf("  Confidence  : %d%%\n",    pkt->confidence);
    Serial.printf("  Pin         : %d%%\n",    pkt->battery_pct);
    Serial.printf("  Hop         : %d / %d\n", pkt->hop_count, pkt->hop_limit);

    Serial.println("  -- Vi tri --");
    if (pkt->position_is_absolute) {
        Serial.printf("  GPS FIX  : lat = %.6f, lon = %.6f\n", pkt->lat, pkt->lon);
        Serial.printf("  Google Maps: https://maps.google.com/?q=%.6f,%.6f\n",
                       pkt->lat, pkt->lon);
    } else {
        Serial.println("  Khong co GPS fix - chi co vi tri PDR tuong doi");
    }
    Serial.printf("  PDR (tuong doi P0): x = %.1f m, y = %.1f m\n",
                   pkt->rel_x_m, pkt->rel_y_m);

    float dist_from_p0 = sqrtf(pkt->rel_x_m * pkt->rel_x_m + pkt->rel_y_m * pkt->rel_y_m);
    Serial.printf("  Khoang cach tu P0: %.1f m\n", dist_from_p0);

    print_separator();
    Serial.println();

    /* ─── Cảnh báo buzzer ─── */
    if (pkt->packet_type == PKT_TYPE_SOS) {
        buzzer_alert(5);  /* 5 tiếng bíp liên tục cho SOS */
    } else {
        buzzer_alert(1);
    }

    /* ─── Cập nhật OLED ─── */
#ifdef USE_OLED
    update_oled(pkt);
#endif
}

/* ═══════════════════════════════════════════════════════════════════
 *  OLED display (tuỳ chọn)
 * ═══════════════════════════════════════════════════════════════════ */

#ifdef USE_OLED
void update_oled(const sos_packet_t *pkt) {
    if (!oled_ok) return;

    oled_display.clearDisplay();
    oled_display.setTextColor(SSD1306_WHITE);

    /* Dòng 1: Loại gói */
    oled_display.setTextSize(2);
    oled_display.setCursor(0, 0);
    if (pkt->packet_type == PKT_TYPE_SOS) {
        oled_display.print("!! SOS !!");
    } else {
        oled_display.print(pkt_type_name(pkt->packet_type));
    }

    oled_display.setTextSize(1);

    /* Dòng 2: Source ID */
    oled_display.setCursor(0, 18);
    char id_buf[20];
    snprintf(id_buf, sizeof(id_buf), "ID: 0x%08lX", (unsigned long)pkt->source_id);
    oled_display.print(id_buf);

    /* Dòng 3: Vị trí */
    oled_display.setCursor(0, 28);
    if (pkt->position_is_absolute) {
        char pos_buf[24];
        snprintf(pos_buf, sizeof(pos_buf), "%.4f, %.4f", pkt->lat, pkt->lon);
        oled_display.print(pos_buf);
    } else {
        char pdr_buf[24];
        snprintf(pdr_buf, sizeof(pdr_buf), "PDR: %.0fm, %.0fm", pkt->rel_x_m, pkt->rel_y_m);
        oled_display.print(pdr_buf);
    }

    /* Dòng 4: Pin + Hop */
    oled_display.setCursor(0, 38);
    char bat_buf[24];
    snprintf(bat_buf, sizeof(bat_buf), "Batt:%d%% Hop:%d/%d",
             pkt->battery_pct, pkt->hop_count, pkt->hop_limit);
    oled_display.print(bat_buf);

    /* Dòng 5: Khoảng cách từ P0 */
    oled_display.setCursor(0, 48);
    float dist = sqrtf(pkt->rel_x_m * pkt->rel_x_m + pkt->rel_y_m * pkt->rel_y_m);
    char dist_buf[24];
    snprintf(dist_buf, sizeof(dist_buf), "Dist P0: %.0f m", dist);
    oled_display.print(dist_buf);

    /* Dòng 6: Số lần nhận */
    oled_display.setCursor(0, 57);
    char cnt_buf[24];
    snprintf(cnt_buf, sizeof(cnt_buf), "RX: %lu", total_rx_count);
    oled_display.print(cnt_buf);

    oled_display.display();
}

void oled_standby() {
    if (!oled_ok) return;
    oled_display.clearDisplay();
    oled_display.setTextSize(1);
    oled_display.setTextColor(SSD1306_WHITE);
    oled_display.setCursor(10, 0);
    oled_display.println("SOS RECEIVER");
    oled_display.setCursor(10, 12);
    oled_display.println("Dang cho...");
    oled_display.setCursor(0, 28);

    if (has_received) {
        uint32_t ago = (millis() - last_rx_time) / 1000;
        char buf[32];
        snprintf(buf, sizeof(buf), "Lan cuoi: %lus truoc", ago);
        oled_display.println(buf);
        char cnt[20];
        snprintf(cnt, sizeof(cnt), "Tong: %lu goi", total_rx_count);
        oled_display.setCursor(0, 40);
        oled_display.println(cnt);
    } else {
        oled_display.println("Chua nhan goi nao");
    }

    oled_display.display();
}
#endif

/* ═══════════════════════════════════════════════════════════════════
 *  SETUP
 * ═══════════════════════════════════════════════════════════════════ */

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    print_separator();
    Serial.println("  SOS RECEIVER - ESP32-S3 + AS32 LoRa");
    Serial.printf("  Packet size: %d bytes\n", sizeof(sos_packet_t));
    print_separator();
    Serial.println();

    /* ─── UART cho AS32 ─── */
    Serial1.begin(AS32_BAUD, SERIAL_8N1, AS32_RX_PIN, AS32_TX_PIN);
    Serial.printf("AS32 UART init: baud=%d, RX=GPIO%d, TX=GPIO%d\n",
                   AS32_BAUD, AS32_RX_PIN, AS32_TX_PIN);

    /* ─── Buzzer ─── */
#ifdef USE_BUZZER
    pinMode(BUZZER_PIN, OUTPUT);
    buzzer_alert(2);
    Serial.println("Buzzer init OK");
#endif

    /* ─── OLED ─── */
#ifdef USE_OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    if (oled_display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oled_ok = true;
        oled_standby();
        Serial.println("OLED init OK");
    } else {
        Serial.println("OLED init FAILED - tiep tuc khong co man hinh");
    }
#endif

    Serial.println("\nSan sang nhan goi SOS...\n");
}

/* ═══════════════════════════════════════════════════════════════════
 *  LOOP
 * ═══════════════════════════════════════════════════════════════════ */

static uint8_t rx_buf[256];
static size_t  rx_len = 0;
static uint32_t last_byte_time = 0;

void loop() {
    /* ─── Đọc bytes từ AS32 UART ─── */
    while (Serial1.available()) {
        if (rx_len < sizeof(rx_buf)) {
            rx_buf[rx_len++] = Serial1.read();
        } else {
            Serial1.read();
        }
        last_byte_time = millis();
    }

    /* ─── Khi đủ bytes cho 1 packet, thử parse ─── */
    if (rx_len >= sizeof(sos_packet_t)) {
        bool found = false;
        for (size_t i = 0; i <= rx_len - sizeof(sos_packet_t); i++) {
            sos_packet_t pkt;
            memcpy(&pkt, &rx_buf[i], sizeof(sos_packet_t));

            /* Kiểm tra tính hợp lệ cơ bản */
            if (pkt.packet_type <= PKT_TYPE_REPEATER_FORWARD &&
                pkt.hop_count <= pkt.hop_limit &&
                pkt.hop_limit <= 15 &&
                pkt.battery_pct <= 100)
            {
                handle_packet(&pkt);

                size_t consumed = i + sizeof(sos_packet_t);
                memmove(rx_buf, &rx_buf[consumed], rx_len - consumed);
                rx_len -= consumed;
                found = true;
                break;
            }
        }

        if (!found && rx_len > sizeof(sos_packet_t) * 2) {
            rx_len = 0;
            Serial.println("Buffer reset - du lieu khong hop le");
        }
    }

    /* ─── Timeout: nhận dở dang quá 2s → reset ─── */
    if (rx_len > 0 && rx_len < sizeof(sos_packet_t) &&
        millis() - last_byte_time > 2000) {
        Serial.printf("Timeout - nhan %d/%d bytes, reset\n",
                       rx_len, sizeof(sos_packet_t));
        rx_len = 0;
    }

    /* ─── Cập nhật OLED standby mỗi 2 giây ─── */
#ifdef USE_OLED
    static uint32_t last_oled_update = 0;
    if (millis() - last_oled_update > 2000) {
        if (!has_received || millis() - last_rx_time > 5000) {
            oled_standby();
        }
        last_oled_update = millis();
    }
#endif

    delay(10);
}
