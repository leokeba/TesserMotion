#include "tessermotion/adxl345.h"

namespace tessermotion {

namespace {
constexpr uint8_t kDevId = 0x00;
constexpr uint8_t kBwRate = 0x2C;
constexpr uint8_t kPowerCtl = 0x2D;
constexpr uint8_t kDataFormat = 0x31;
constexpr uint8_t kDataX0 = 0x32;  // X, Y, Z, little-endian
constexpr uint8_t kExpectedId = 0xE5;
constexpr float kMs2PerLsb = 0.0039f * 9.80665f;  // 3.9 mg in full resolution

bool writeReg(I2c& bus, uint8_t addr, uint8_t reg, uint8_t value) { return bus.write(addr, reg, &value, 1); }
}  // namespace

bool Adxl345::begin() {
    if (!bus_.begin()) return false;
    uint8_t id = 0;
    if (!bus_.read(address_, kDevId, &id, 1) || id != kExpectedId) return false;
    failures_ = 0;
    return writeReg(bus_, address_, kBwRate, 0x0A)          // 100 Hz
           && writeReg(bus_, address_, kDataFormat, 0x0B)   // full resolution, ±16 g
           && writeReg(bus_, address_, kPowerCtl, 0x08);    // measure
}

bool Adxl345::read(float& x, float& y, float& z) {
    uint8_t b[6];
    if (!bus_.read(address_, kDataX0, b, sizeof b)) {
        if (++failures_ >= kRecoverAfter) {
            bus_.recover();
            failures_ = 0;
        }
        return false;
    }
    failures_ = 0;
    auto axis = [&b](int i) { return static_cast<float>(static_cast<int16_t>(b[i] | (b[i + 1] << 8))) * kMs2PerLsb; };
    x = axis(0);
    y = axis(2);
    z = axis(4);
    return true;
}

}  // namespace tessermotion
