#include "core/fusion/calibration_data_log.h"
#include <time.h>
#include <string.h>

// Generate a filename with a timestamp: e.g. "gyro_calib_20250513_210112.csv"
void make_calib_filename(char *buf, size_t buflen, const char *prefix, uint64_t unix_time) {
    time_t t = (time_t)(unix_time / 1000000ULL); // Convert microseconds to seconds
    struct tm *tm_info = localtime(&t);
    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%Y%m%d_%H%M%S", tm_info);
    snprintf(buf, buflen, "%s_calib_%s.csv", prefix, timebuf);
}
