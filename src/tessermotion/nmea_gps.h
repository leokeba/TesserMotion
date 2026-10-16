#pragma once

// A GPS receiver speaking NMEA on a UART (docs/DESIGN.md section 6.2), such as
// a u-blox NEO at 9600 baud. A Part: it reads the UART every tick without
// blocking and parses there, so add it to a Motion.

#include <stdint.h>

#include <string>

#include "tessermotion/motion.h"
#include "tessermotion/nmea.h"
#include "tessermotion/platform.h"

namespace tessermotion {

// GGA's fix quality.
enum class GpsFix : uint8_t { None, Gps, Dgps, Pps, Rtk, FloatRtk, Estimated, Manual, Simulation };

class NmeaGps : public Part {
public:
    // Without a GGA for this long (the receiver unplugged or silent), the
    // fix is none.
    static constexpr uint32_t kStaleMs = 3000;

    struct Fix {
        GpsFix quality = GpsFix::None;
        uint8_t satellites = 0;
        float hdop = 0;
        // The last position received with a fix, kept when the fix is lost.
        double latitude = 0, longitude = 0;  // degrees, north and east positive
        float altitude = 0;                  // m above mean sea level
        int32_t ageMs = -1;                  // since that position; -1: none yet
    };

    explicit NmeaGps(Uart& uart) : uart_(uart) {}

    // From any task: these take Motion's lock.
    Fix fix();
    // The UTC time of the latest valid RMC with a date, advanced to now
    // (ms since 1970), once per sentence: false when there's nothing new.
    // Poll it from the application's task, not from the control task's
    // callbacks (DESIGN section 6.2).
    bool takeTime(int64_t& epochMs);
    // The latest valid RMC's time, ISO 8601 ("2026-10-16T12:34:56Z"); empty
    // before one came.
    std::string utc();
    // Sentences dropped: checksums, malformed or cut short.
    uint32_t errors();

    // Part
    void begin() override;
    void sense(uint64_t nowUs) override;
    void update(uint64_t nowUs) override;

private:
    Uart& uart_;
    bool started_ = false;
    Nmea nmea_;

    // Sensed outside the lock, by the control task only: what the parser
    // completed since the last update().
    bool gotGga_ = false, gotRmc_ = false;

    // Under the lock.
    Fix fix_;
    uint64_t lastGgaUs_ = 0;
    uint64_t fixUs_ = 0;  // when the position in fix_ came; 0: none
    bool hasTime_ = false, timeTaken_ = true;
    int64_t timeMs_ = 0;  // the latest valid RMC's
    uint64_t timeUs_ = 0;  // when it came
    uint32_t errors_ = 0;
};

}  // namespace tessermotion
