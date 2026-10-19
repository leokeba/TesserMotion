#include "tessermotion/tmc5160.h"

#include <math.h>

#include "tessermotion/tmc/tmc5160_map.h"

namespace tessermotion {

namespace {

namespace t = tmc5160;

constexpr TmcDriver::Common kCommon = {
    t::version, Tmc5160::kVersion, t::reset, t::drv_err, t::shaft, t::tpwmthrs,
    t::toff, t::hstrt, t::hend, t::tbl, t::mres, t::intpol,
    t::stst, t::stealth, t::cs_actual, t::ot, t::otpw, t::s2ga, t::s2gb, t::s2vsa, t::s2vsb, t::ola, t::olb,
    t::sg_result,
    t::pwm_ofs, t::pwm_grad, t::pwm_freq, t::pwm_autoscale, t::pwm_autograd, t::freewheel, t::pwm_reg, t::pwm_lim,
};

// PWMCONF's reset value, 0xC40C001E.
constexpr TmcDriver::Pwm kPwm = {30, 0, 0, true, true, 0, 4, 12};

// Read every tick: faults and resets, the driver's state, the position and
// the velocity.
constexpr uint8_t kFast[] = {t::GSTAT, t::DRV_STATUS, t::XACTUAL, t::VACTUAL};

constexpr float kFullScaleVolts = 0.325f;  // across the sense resistor at GLOBAL_SCALER 256, CS 31
constexpr float kVmax = 8388096;           // 2^23 − 512
constexpr int32_t kVstop = 10;             // the datasheet's suggestion; not 0 when positioning

int32_t rounded(float v, float hi) {
    if (!(v > 0)) return 0;
    return static_cast<int32_t>(lroundf(v < hi ? v : hi));
}

}  // namespace

Tmc5160::Tmc5160(Spi& bus, const Tmc5160Config& config, Gpio* gpio)
    : TmcDriver(t::kMap, kCommon, 256, kPwm, config.clockHz, config.enable, gpio),
      bus_(bus),
      config_(config),
      device_{config.cs, 3, config.spiHz} {}

float Tmc5160::maxCurrent() const { return kFullScaleVolts / (1.41421356f * config_.rSense); }

bool Tmc5160::busBegin() { return bus_.begin(); }

// A datagram is 40 bits: the address (bit 7 set to write) and 32 data bits.
// The reply carries the status byte, then the data the previous datagram
// asked for.
bool Tmc5160::transfer(uint8_t first, uint32_t data, uint32_t& reply) {
    uint8_t tx[5] = {first, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16),
                     static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)};
    uint8_t rx[5] = {};
    if (!bus_.transfer(device_, tx, rx, sizeof tx)) {
        busError_ = TmcError::Bus;
        return false;
    }
    reply = (uint32_t(rx[1]) << 24) | (uint32_t(rx[2]) << 16) | (uint32_t(rx[3]) << 8) | rx[4];
    // A floating or missing MISO.
    if (rx[0] == 0xFF && reply == 0xFFFFFFFFu) {
        busError_ = TmcError::NoReply;
        return false;
    }
    return true;
}

bool Tmc5160::busRead(uint8_t address, uint32_t& value) {
    uint32_t ignored;
    return transfer(address & 0x7F, 0, ignored) && transfer(address & 0x7F, 0, value);
}

bool Tmc5160::busWrite(uint8_t address, uint32_t value) {
    uint32_t ignored;
    // A write's reply is about the previous datagram: no answer to check.
    transfer(static_cast<uint8_t>(address | 0x80), value, ignored);
    return true;
}

size_t Tmc5160::fastRegisters(const uint8_t*& list) const {
    list = kFast;
    return sizeof kFast;
}

bool Tmc5160::drives() const { return get(t::sd_mode) == 0; }

uint8_t Tmc5160::velocityRegister() const { return t::VMAX; }

bool Tmc5160::canChangeResolution() const { return !drives() || get(t::vactual) == 0; }

float Tmc5160::measuredVelocity() const {
    return static_cast<float>(get(t::vactual)) * static_cast<float>(clockHz()) / 16777216.0f;
}

void Tmc5160::applyChip() {
    // I = GLOBAL_SCALER/256 · (CS + 1)/32 · 0.325 V / R / √2. The scaler
    // takes the current, CS 31 at the run current: its 1..31 aren't allowed.
    float run = runAmps();
    float hold = holdAmps();
    float full = 256.0f * run * 1.41421356f * config_.rSense / kFullScaleVolts;
    int32_t scaler = static_cast<int32_t>(ceilf(full));
    scaler = scaler < 32 ? 32 : scaler > 256 ? 256 : scaler;
    float scale = static_cast<float>(scaler) / 256.0f;
    set(t::global_scaler, scaler == 256 ? 0 : scaler);
    set(t::irun, currentScale(run, config_.rSense, kFullScaleVolts, scale));
    set(t::ihold, currentScale(hold, config_.rSense, kFullScaleVolts, scale));
    set(t::en_pwm_mode, settings.stealthChop);

    // A trapezoid: AMAX up and DMAX down, no second phase (V1 0).
    // a[µsteps/s²] = AMAX · fCLK² / 2^41.
    float clock = static_cast<float>(clockHz());
    float a = ramp.acceleration * 2199023255552.0f / (clock * clock);
    int32_t amax = rounded(a, 65535);
    if (amax < 1) amax = 1;  // D1 and DMAX can't be 0 when positioning
    set(t::vstart, 0);
    set(t::v1, 0);
    set(t::a1, amax);
    set(t::amax, amax);
    set(t::dmax, amax);
    set(t::d1, amax);
    set(t::vstop, kVstop);
}

void Tmc5160::applyMotion() {
    int32_t vmax = rounded(velocityUnits(ramp.maxSpeed), kVmax);
    if (!enabled_ || drive_ == Drive::Hold) {
        // Velocity mode at 0: decelerates and stays, wherever it was.
        set(t::rampmode, 1);
        set(t::vmax, 0);
    } else if (drive_ == Drive::Velocity) {
        set(t::rampmode, velocity_ >= 0 ? 1 : 2);
        int32_t v = rounded(velocityUnits(fabsf(velocity_)), kVmax);
        set(t::vmax, v < vmax ? v : vmax);
    } else {
        set(t::rampmode, 0);
        set(t::vmax, vmax);
        set(t::xtarget, target_);
    }
}

bool Tmc5160::setTarget(int32_t microsteps) {
    Guard g(lock());
    if (!enabled_ || !present() || !drives()) return false;
    target_ = microsteps;
    drive_ = Drive::Position;
    velocity_ = 0;
    return true;
}

int32_t Tmc5160::target() {
    Guard g(lock());
    return drive_ == Drive::Position ? target_ : get(t::xactual);
}

int32_t Tmc5160::position() {
    Guard g(lock());
    return get(t::xactual);
}

bool Tmc5160::stepDir() {
    Guard g(lock());
    return !drives();
}

}  // namespace tessermotion
