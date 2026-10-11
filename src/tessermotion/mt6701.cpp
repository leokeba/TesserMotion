#include "tessermotion/mt6701.h"

namespace tessermotion {

namespace {
constexpr uint8_t kAngleHigh = 0x03;  // angle bits 13..6; 0x04 holds bits 5..0 in its bits 7..2
}

bool Mt6701::begin() { return bus_.begin(); }

bool Mt6701::read(float& degrees) {
    uint8_t b[2];
    if (!bus_.read(kAddress, kAngleHigh, b, sizeof b)) {
        if (++failures_ >= kRecoverAfter) {
            bus_.recover();
            failures_ = 0;
        }
        return false;
    }
    failures_ = 0;
    uint32_t counts = (uint32_t(b[0]) << 6) | (uint32_t(b[1]) >> 2);
    degrees = static_cast<float>(counts) * (360.0f / 16384.0f);
    return true;
}

}  // namespace tessermotion
