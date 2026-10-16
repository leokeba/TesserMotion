#include "tessermotion/nmea.h"

#include <string.h>

namespace tessermotion {

namespace {

bool isDigit(char c) { return c >= '0' && c <= '9'; }

int hexDigit(char c) {
    if (isDigit(c)) return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// `n` decimal digits.
bool digits(const char* s, int n, int& out) {
    out = 0;
    for (int i = 0; i < n; i++) {
        if (!isDigit(s[i])) return false;
        out = out * 10 + (s[i] - '0');
    }
    return true;
}

// [-]digits[.digits], at least one digit; no exponent, no locale.
bool number(const char* s, double& out) {
    bool negative = *s == '-';
    if (negative) s++;
    double v = 0, scale = 1;
    bool any = false, point = false;
    for (; *s; s++) {
        if (isDigit(*s)) {
            any = true;
            if (point) {
                scale /= 10;
                v += (*s - '0') * scale;
            } else {
                v = v * 10 + (*s - '0');
            }
        } else if (*s == '.' && !point) {
            point = true;
        } else {
            return false;
        }
    }
    out = negative ? -v : v;
    return any;
}

bool unsignedInt(const char* s, uint32_t max, uint32_t& out) {
    if (!*s) return false;
    out = 0;
    for (; *s; s++) {
        if (!isDigit(*s)) return false;
        out = out * 10 + static_cast<uint32_t>(*s - '0');
        if (out > max) return false;
    }
    return true;
}

// hhmmss[.sss]
bool timeOfDay(const char* s, int64_t& ms) {
    int h, m, sec;
    if (strlen(s) < 6 || !digits(s, 2, h) || !digits(s + 2, 2, m) || !digits(s + 4, 2, sec)) return false;
    if (h > 23 || m > 59 || sec > 60) return false;
    int frac = 0, scale = 100;
    const char* p = s + 6;
    if (*p == '.') {
        for (p++; *p; p++) {
            if (!isDigit(*p)) return false;
            frac += (*p - '0') * scale;
            scale /= 10;
        }
    } else if (*p) {
        return false;
    }
    ms = ((h * 60 + m) * 60 + sec) * int64_t(1000) + frac;
    return true;
}

// ddmmyy
bool date(const char* s, int& year, int& month, int& day) {
    int yy;
    if (strlen(s) != 6 || !digits(s, 2, day) || !digits(s + 2, 2, month) || !digits(s + 4, 2, yy)) return false;
    if (day < 1 || day > 31 || month < 1 || month > 12) return false;
    year = yy < 80 ? 2000 + yy : 1900 + yy;
    return true;
}

// (d)ddmm.mmmm and its hemisphere.
bool coordinate(const char* value, const char* hemisphere, bool latitude, double& out) {
    double v;
    if (!number(value, v) || v < 0) return false;
    double deg = static_cast<double>(static_cast<int>(v / 100));
    double min = v - deg * 100;
    if (min >= 60 || deg > (latitude ? 90 : 180)) return false;
    char positive = latitude ? 'N' : 'E', negative = latitude ? 'S' : 'W';
    if (hemisphere[1] || (hemisphere[0] != positive && hemisphere[0] != negative)) return false;
    out = deg + min / 60;
    if (hemisphere[0] == negative) out = -out;
    return true;
}

}  // namespace

int64_t Nmea::epochMs(int year, int month, int day, int64_t msOfDay) {
    // Days from the civil date (H. Hinnant's algorithm).
    int y = year - (month <= 2 ? 1 : 0);
    int era = (y >= 0 ? y : y - 399) / 400;
    int yoe = y - era * 400;
    int doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = int64_t(era) * 146097 + doe - 719468;
    return days * 86400000 + msOfDay;
}

Nmea::Sentence Nmea::error() {
    errors_++;
    return Sentence::Error;
}

Nmea::Sentence Nmea::feed(char c) {
    if (c == '$') {
        // A sentence cut short by the next one.
        Sentence s = inLine_ ? error() : Sentence::None;
        line_[0] = '$';
        len_ = 1;
        inLine_ = true;
        return s;
    }
    if (!inLine_) return Sentence::None;  // waiting for a '$'
    if (c == '\r' || c == '\n') {
        inLine_ = false;
        return parse();
    }
    if (len_ == kMaxLine) {
        inLine_ = false;
        return error();
    }
    line_[len_++] = c;
    return Sentence::None;
}

Nmea::Sentence Nmea::parse() {
    // $<fields>*hh
    if (len_ < 4 || line_[len_ - 3] != '*') return error();
    int hi = hexDigit(line_[len_ - 2]), lo = hexDigit(line_[len_ - 1]);
    if (hi < 0 || lo < 0) return error();
    uint8_t sum = 0;
    for (size_t i = 1; i < len_ - 3; i++) sum = static_cast<uint8_t>(sum ^ static_cast<uint8_t>(line_[i]));
    if (sum != hi * 16 + lo) return error();
    line_[len_ - 3] = '\0';

    const char* f[kMaxFields];
    size_t n = 0;
    f[n++] = line_ + 1;
    for (char* p = line_ + 1; *p; p++) {
        if (*p != ',') continue;
        *p = '\0';
        if (n == kMaxFields) break;
        f[n++] = p + 1;
    }
    // The type follows a two-letter talker (GP, GN, GL...); proprietary
    // sentences (PUBX...) are other.
    if (strlen(f[0]) != 5 || f[0][0] == 'P') return Sentence::Other;
    const char* type = f[0] + 2;

    if (strcmp(type, "RMC") == 0) {
        // time, status, lat, N/S, lon, E/W, speed, course, date[, magnetic
        // variation, E/W, mode]
        if (n < 10) return error();
        Rmc r;
        if (strcmp(f[2], "A") == 0) {
            r.valid = true;
        } else if (strcmp(f[2], "V") != 0) {
            return error();
        }
        if (n > 12 && strcmp(f[12], "N") == 0) r.valid = false;  // mode: not valid
        if (r.valid) {
            int64_t ms;
            int y, mo, d;
            if (*f[1] && *f[9]) {
                if (!timeOfDay(f[1], ms) || !date(f[9], y, mo, d)) return error();
                r.hasDate = true;
                r.epochMs = epochMs(y, mo, d, ms);
            }
            if (*f[3]) {
                if (!coordinate(f[3], f[4], true, r.latitude) || !coordinate(f[5], f[6], false, r.longitude)) {
                    return error();
                }
                r.hasPosition = true;
            }
        }
        rmc_ = r;
        return Sentence::Rmc;
    }
    if (strcmp(type, "GGA") == 0) {
        // time, lat, N/S, lon, E/W, quality, satellites, hdop, altitude, M, ...
        if (n < 10) return error();
        Gga g;
        uint32_t u = 0;
        if (*f[6] && !unsignedInt(f[6], 8, u)) return error();
        g.quality = static_cast<uint8_t>(u);
        u = 0;
        if (*f[7] && !unsignedInt(f[7], 99, u)) return error();
        g.satellites = static_cast<uint8_t>(u);
        double v;
        if (*f[8]) {
            if (!number(f[8], v)) return error();
            g.hdop = static_cast<float>(v);
        }
        if (g.quality > 0) {
            if (!coordinate(f[2], f[3], true, g.latitude) || !coordinate(f[4], f[5], false, g.longitude)) {
                return error();
            }
            if (*f[9]) {
                if (!number(f[9], v)) return error();
                g.altitude = static_cast<float>(v);
            }
            g.hasPosition = true;
        }
        gga_ = g;
        return Sentence::Gga;
    }
    return Sentence::Other;
}

}  // namespace tessermotion
