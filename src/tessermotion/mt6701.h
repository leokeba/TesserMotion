#pragma once

// The MT6701 magnetic angle encoder over I2C (docs/DESIGN.md section 4.2).
// Its address is fixed, so two encoders need two buses.

#include <stdint.h>

#include "tessermotion/angle_sensor.h"
#include "tessermotion/platform.h"

namespace tessermotion {

class Mt6701 : public AngleSensor {
public:
    static constexpr uint8_t kAddress = 0x06;
    // Failed reads in a row before the bus is recovered.
    static constexpr uint8_t kRecoverAfter = 3;

    explicit Mt6701(I2c& bus) : bus_(bus) {}

    bool begin() override;
    bool read(float& degrees) override;

private:
    I2c& bus_;
    uint8_t failures_ = 0;
};

}  // namespace tessermotion
