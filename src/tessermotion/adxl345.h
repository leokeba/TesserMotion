#pragma once

// The ADXL345 accelerometer over I2C (docs/DESIGN.md section 6.1): 100 Hz,
// full resolution, ±16 g. It may share its bus with other devices, such as an
// MT6701.

#include <stdint.h>

#include "tessermotion/accelerometer.h"
#include "tessermotion/platform.h"

namespace tessermotion {

class Adxl345 : public Accelerometer {
public:
    static constexpr uint8_t kAddress = 0x53;     // SDO low; 0x1D with SDO high
    static constexpr uint8_t kRecoverAfter = 3;   // failed reads in a row before the bus is recovered

    explicit Adxl345(I2c& bus, uint8_t address = kAddress) : bus_(bus), address_(address) {}

    // Checks the device ID, then configures it. Never recovers the bus: an
    // absent sensor only NACKs.
    bool begin() override;
    bool read(float& x, float& y, float& z) override;

private:
    I2c& bus_;
    uint8_t address_;
    uint8_t failures_ = 0;
};

}  // namespace tessermotion
