#include "tessermotion/nmea_gps.h"

#include <stdio.h>

namespace tessermotion {

namespace {

// Year, month, day of days since 1970 (H. Hinnant's algorithm).
void civil(int64_t days, int& year, int& month, int& day) {
    days += 719468;
    int64_t era = (days >= 0 ? days : days - 146096) / 146097;
    int doe = static_cast<int>(days - era * 146097);
    int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int mp = (5 * doy + 2) / 153;
    day = doy - (153 * mp + 2) / 5 + 1;
    month = mp < 10 ? mp + 3 : mp - 9;
    year = static_cast<int>(yoe + era * 400) + (month <= 2 ? 1 : 0);
}

}  // namespace

void NmeaGps::begin() { started_ = uart_.begin(); }

void NmeaGps::sense(uint64_t nowUs) {
    (void)nowUs;
    if (!started_) return;
    // What came since the last tick; bounded, should the line be flooded.
    uint8_t buf[64];
    for (int chunk = 0; chunk < 8; chunk++) {
        size_t n = uart_.read(buf, sizeof buf);
        for (size_t i = 0; i < n; i++) {
            Nmea::Sentence s = nmea_.feed(static_cast<char>(buf[i]));
            if (s == Nmea::Sentence::Gga) gotGga_ = true;
            if (s == Nmea::Sentence::Rmc) gotRmc_ = true;
        }
        if (n < sizeof buf) break;
    }
}

void NmeaGps::update(uint64_t nowUs) {
    if (gotGga_) {
        const Nmea::Gga& g = nmea_.gga();
        lastGgaUs_ = nowUs;
        fix_.quality = static_cast<GpsFix>(g.quality);
        fix_.satellites = g.satellites;
        fix_.hdop = g.hdop;
        if (g.hasPosition) {
            fix_.latitude = g.latitude;
            fix_.longitude = g.longitude;
            fix_.altitude = g.altitude;
            fixUs_ = nowUs;
        }
    }
    if (gotRmc_) {
        const Nmea::Rmc& r = nmea_.rmc();
        if (r.valid && r.hasPosition) {
            fix_.latitude = r.latitude;
            fix_.longitude = r.longitude;
            fixUs_ = nowUs;
        }
        if (r.valid && r.hasDate) {
            timeMs_ = r.epochMs;
            timeUs_ = nowUs;
            hasTime_ = true;
            timeTaken_ = false;
        }
    }
    gotGga_ = gotRmc_ = false;
    if (fix_.quality != GpsFix::None && nowUs - lastGgaUs_ > uint64_t(kStaleMs) * 1000) {
        fix_.quality = GpsFix::None;
        fix_.satellites = 0;
        fix_.hdop = 0;
    }
    if (fixUs_) {
        uint64_t ms = (nowUs - fixUs_) / 1000;
        fix_.ageMs = ms > INT32_MAX ? INT32_MAX : static_cast<int32_t>(ms);
    }
    errors_ = nmea_.errors();
}

NmeaGps::Fix NmeaGps::fix() {
    Guard g(lock());
    return fix_;
}

bool NmeaGps::takeTime(int64_t& epochMs) {
    Guard g(lock());
    if (!hasTime_ || timeTaken_) return false;
    uint64_t now = nowUs();
    epochMs = timeMs_ + static_cast<int64_t>(now > timeUs_ ? (now - timeUs_) / 1000 : 0);
    timeTaken_ = true;
    return true;
}

std::string NmeaGps::utc() {
    int64_t ms;
    {
        Guard g(lock());
        if (!hasTime_) return std::string();
        ms = timeMs_;
    }
    int64_t s = ms / 1000;
    int year, month, day;
    civil(s / 86400, year, month, day);
    int sod = static_cast<int>(s % 86400);
    char buf[48];
    snprintf(buf, sizeof buf, "%04d-%02d-%02dT%02d:%02d:%02dZ", year, month, day, sod / 3600, sod / 60 % 60, sod % 60);
    return buf;
}

uint32_t NmeaGps::errors() {
    Guard g(lock());
    return errors_;
}

}  // namespace tessermotion
