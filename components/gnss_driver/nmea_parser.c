#include "nmea_parser.h"
#include <string.h>
#include <stdlib.h>

/* GY-NEO7M (u-blox NEO-7M) xuất mặc định các câu: GGA, GLL, GSA, GSV, RMC, VTG @9600 baud.
 * Ở đây ta chỉ cần GGA (vị trí + HDOP + số vệ tinh) là đủ cho P0. */

static double nmea_to_decimal_deg(const char *raw, char hemi)
{
    /* TODO: chuyển ddmm.mmmm / dddmm.mmmm -> độ thập phân,
     * đảo dấu nếu hemi == 'S' hoặc 'W'. */
    (void)raw; (void)hemi;
    return 0.0;
}

bool nmea_parse_line(const char *line, nmea_fix_t *fix)
{
    if (!line || !fix) return false;

    if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
        /* TODO: tách các trường theo dấu phẩy:
         * $GxGGA,time,lat,N/S,lon,E/W,fixQuality,numSat,HDOP,alt,... 
         * fixQuality == 0 nghĩa là chưa có fix -> fix->valid = false. */
        fix->valid = false;
        return false;
    }

    /* Có thể bổ sung parse $GxRMC nếu cần tốc độ/hướng di chuyển từ GNSS. */
    return false;
}
