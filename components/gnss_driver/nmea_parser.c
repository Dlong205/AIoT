#include "nmea_parser.h"
#include <string.h>
#include <stdlib.h>

static double nmea_to_decimal_deg(const char *raw, char hemi)
{
    if (!raw || raw[0] == '\0') return 0.0;

    double val = atof(raw);
    int deg = (int)(val / 100);
    double min = val - (deg * 100);
    double dec = deg + (min / 60.0);

    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}

static int split_fields(char *line, char *fields[], int max_fields)
{
    int count = 0;
    char *p = line;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    return count;
}

bool nmea_parse_line(const char *line, nmea_fix_t *fix)
{
    if (!line || !fix) return false;

    if (strncmp(line, "$GPGGA", 6) != 0 && strncmp(line, "$GNGGA", 6) != 0) {
        return false;
    }

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[20] = {0};
    int n = split_fields(buf, fields, 20);
    if (n < 9) {
        fix->valid = false;
        return false;
    }

    int fix_quality = atoi(fields[6]);
    if (fix_quality == 0) {
        fix->valid = false;
        return false;
    }

    fix->lat = nmea_to_decimal_deg(fields[2], fields[3][0]);
    fix->lon = nmea_to_decimal_deg(fields[4], fields[5][0]);
    fix->satellites = (uint8_t)atoi(fields[7]);
    fix->hdop = (float)atof(fields[8]);
    fix->valid = true;
    return true;
}
