#pragma once

// An accelerometer read by the control task and averaged (docs/DESIGN.md
// section 6.1): x, y, z, and the pitch and roll they give. A Part.
//
// A sensor that doesn't answer at begin(), or fails 3 reads in a row, is
// absent: it isn't read, and begin() is tried again every 2 s.

#include <stdint.h>

#include "tessermotion/accelerometer.h"
#include "tessermotion/motion.h"

namespace tessermotion {

class Tilt : public Part {
public:
    static constexpr uint32_t kMaxSamples = 32;
    static constexpr uint8_t kAbsentAfter = 3;
    static constexpr uint32_t kRetryMs = 2000;

    struct Reading {
        float x = 0, y = 0, z = 0;  // m/s², averaged
        float pitch = 0;            // °: atan2(−x, √(y² + z²))
        float roll = 0;             // °: atan2(y, z)
    };

    explicit Tilt(Accelerometer& sensor) : sensor_(sensor) {}

    // Settings: set them before Motion starts, or under lock() afterwards
    // (the tree does).
    uint32_t samples = 8;  // averaged, 1..kMaxSamples
    uint32_t every = 5;    // ticks between reads (10 Hz at 50 Hz); before start

    // From any task: these take Motion's lock.
    Reading reading();
    bool present();

    Accelerometer& sensor() { return sensor_; }

    // Part
    void begin() override;
    void sense(uint64_t nowUs) override;
    void update(uint64_t nowUs) override;

private:
    struct Sample {
        float x, y, z;
    };

    Accelerometer& sensor_;

    // The control task's only.
    bool found_ = false;
    uint64_t retryUs_ = 0;
    uint8_t failures_ = 0;
    uint32_t countdown_ = 0;
    bool sampleOk_ = false;
    Sample sample_{0, 0, 0};

    // Under the lock.
    bool present_ = false;
    Sample ring_[kMaxSamples];
    uint32_t head_ = 0, filled_ = 0;
    Reading reading_;
};

}  // namespace tessermotion
