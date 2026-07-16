#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool    valid;
    double  lat;
    double  lon;
    float   hdop;
    uint8_t satellites;
} nmea_fix_t;

/* Parse 1 dòng NMEA ($GxGGA hoặc $GxRMC, GY-NEO7M xuất chuẩn NMEA 0183).
 * Trả true nếu parse ra được fix hợp lệ (mới hoặc cập nhật thêm field). */
bool nmea_parse_line(const char *line, nmea_fix_t *fix);
