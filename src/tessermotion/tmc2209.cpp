#include "tessermotion/tmc2209.h"

#include <math.h>
#include <string.h>

#include "tessermotion/tmc/tmc2209_map.h"

namespace tessermotion {

namespace {

namespace t = tmc2209;

constexpr TmcDriver::Common kCommon = {
    t::version, Tmc2209::kVersion, t::reset, t::drv_err, t::shaft, t::tpwmthrs,
    t::toff, t::hstrt, t::hend, t::tbl, t::mres, t::intpol,
    t::stst, t::stealth, t::cs_actual, t::ot, t::otpw, t::s2ga, t::s2gb, t::s2vsa, t::s2vsb, t::ola, t::olb,
};

// Read every tick, besides one other in turn: GSTAT comes round in a third
// of a second at 50 Hz.
constexpr uint8_t kFast[] = {t::DRV_STATUS};

constexpr uint8_t kSync = 0x05;
constexpr uint8_t kReplyAddress = 0xFF;
constexpr float kInternalOhms = 0.02f;  // added to the sense resistor
constexpr float kFullScaleVolts = 0.325f;
constexpr float kLowScaleVolts = 0.180f;  // with vsense
constexpr float kVactualMax = 8388607;    // 2^23 − 1

}  // namespace

Tmc2209::Tmc2209(Uart& uart, const Tmc2209Config& config, Gpio* gpio)
    : TmcDriver(t::kMap, kCommon, 16, config.clockHz, config.enable, gpio), uart_(uart), config_(config) {}

float Tmc2209::maxCurrent() const { return kFullScaleVolts / (1.41421356f * (config_.rSense + kInternalOhms)); }

uint8_t Tmc2209::crc(const uint8_t* data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int bit = 0; bit < 8; bit++) {
            c = ((c >> 7) ^ (b & 1)) ? static_cast<uint8_t>((c << 1) ^ 0x07) : static_cast<uint8_t>(c << 1);
            b = static_cast<uint8_t>(b >> 1);
        }
    }
    return c;
}

bool Tmc2209::busBegin() { return uart_.begin(); }

void Tmc2209::drain() {
    uint8_t junk[16];
    while (uart_.read(junk, sizeof junk) > 0) {
    }
}

// A read request is 4 bytes: sync, the node's address, the register, the
// CRC. The reply is 8: sync, 0xFF, the register, 32 data bits, the CRC.
bool Tmc2209::busRead(uint8_t address, uint32_t& value) {
    drain();
    uint8_t request[4] = {kSync, config_.address, static_cast<uint8_t>(address & 0x7F), 0};
    request[3] = crc(request, 3);
    if (!uart_.write(request, sizeof request)) return false;
    uint8_t reply[8];
    if (uart_.read(reply, 4, kTimeoutMs) != 4) return false;
    size_t have = 4;
    echo_ = memcmp(reply, request, sizeof request) == 0;
    if (echo_) have = 0;
    if (uart_.read(reply + have, sizeof reply - have, kTimeoutMs) != sizeof reply - have) return false;
    if (reply[0] != kSync || reply[1] != kReplyAddress || reply[2] != request[2] || reply[7] != crc(reply, 7)) {
        return false;
    }
    value = (uint32_t(reply[3]) << 24) | (uint32_t(reply[4]) << 16) | (uint32_t(reply[5]) << 8) | reply[6];
    return true;
}

// A write is 8 bytes, unanswered: sync, address, register | 0x80, data, CRC.
bool Tmc2209::busWrite(uint8_t address, uint32_t value) {
    drain();
    uint8_t d[8] = {kSync,
                    config_.address,
                    static_cast<uint8_t>(address | 0x80),
                    static_cast<uint8_t>(value >> 24),
                    static_cast<uint8_t>(value >> 16),
                    static_cast<uint8_t>(value >> 8),
                    static_cast<uint8_t>(value),
                    0};
    d[7] = crc(d, 7);
    if (!uart_.write(d, sizeof d)) return false;
    // Its echo, so the next request's isn't taken for it.
    if (echo_) {
        uint8_t e[8];
        uart_.read(e, sizeof e, kTimeoutMs);
    }
    return true;
}

size_t Tmc2209::fastRegisters(const uint8_t*& list) const {
    list = kFast;
    return sizeof kFast;
}

void Tmc2209::applyChip() {
    // I = (CS + 1)/32 · V/(R + 20 mΩ)/√2, V 0.325, or 0.180 with vsense:
    // finer steps for currents under half of full scale.
    float r = config_.rSense + kInternalOhms;
    float run = settings.runCurrent;
    float hold = settings.holdCurrent < run ? settings.holdCurrent : run;
    bool vsense = currentScale(run, r, kFullScaleVolts) < 16;
    float volts = vsense ? kLowScaleVolts : kFullScaleVolts;
    set(t::vsense, vsense);
    set(t::irun, currentScale(run, r, volts));
    set(t::ihold, currentScale(hold, r, volts));
    // The UART configures it: not PDN, not the MS pins, not VREF.
    set(t::pdn_disable, 1);
    set(t::mstep_reg_select, 1);
    set(t::i_scale_analog, 0);
    set(t::multistep_filt, 1);
    set(t::en_spreadcycle, !settings.stealthChop);
}

void Tmc2209::applyMotion() {
    float v = 0;
    if (enabled_ && drive_ == Drive::Velocity) v = velocityUnits(velocity_);
    v = v > kVactualMax ? kVactualMax : v < -kVactualMax ? -kVactualMax : v;
    set(t::vactual, static_cast<int32_t>(lroundf(v)));
}

}  // namespace tessermotion
