#pragma once

// An NMEA 0183 parser for RMC and GGA sentences, any talker (docs/DESIGN.md
// section 6.2). Bytes go in one at a time; each completed sentence updates
// rmc() or gga(). Lines with a bad or missing checksum, malformed fields or
// too long are counted in errors() and dropped. Other sentences are ignored.

#include <stddef.h>
#include <stdint.h>

namespace tessermotion {

class Nmea {
public:
    enum class Sentence : uint8_t { None, Rmc, Gga, Other, Error };

    // The latest RMC.
    struct Rmc {
        bool valid = false;  // status A: the receiver has a fix
        bool hasDate = false;
        int64_t epochMs = 0;  // UTC, when valid and hasDate
        bool hasPosition = false;
        double latitude = 0, longitude = 0;  // degrees, north and east positive
    };
    // The latest GGA.
    struct Gga {
        uint8_t quality = 0;  // 0 no fix, 1 GPS, 2 DGPS, ... 8 simulation
        uint8_t satellites = 0;
        float hdop = 0;  // 0 when not given
        bool hasPosition = false;  // with quality > 0
        double latitude = 0, longitude = 0;
        float altitude = 0;  // m above mean sea level
    };

    // Feeds one received byte; returns the sentence it completed, if any.
    Sentence feed(char c);

    const Rmc& rmc() const { return rmc_; }
    const Gga& gga() const { return gga_; }
    uint32_t errors() const { return errors_; }

    // Milliseconds since 1970 of a UTC date and time.
    static int64_t epochMs(int year, int month, int day, int64_t msOfDay);

private:
    static constexpr size_t kMaxLine = 100;  // NMEA's limit is 82, with "$" and CR LF
    static constexpr size_t kMaxFields = 24;

    Sentence parse();
    Sentence error();

    char line_[kMaxLine + 1];
    size_t len_ = 0;
    bool inLine_ = false;
    Rmc rmc_;
    Gga gga_;
    uint32_t errors_ = 0;
};

}  // namespace tessermotion
