#include "tessermotion/tmc_driver.h"

#include <math.h>
#include <string.h>

namespace tessermotion {

namespace {

// A count of microsteps as one the chip has: a power of 2 from 1 to 256,
// the lower one between two.
uint16_t powerOfTwo(uint16_t microsteps) {
    uint16_t m = 1;
    while (m < 256 && m * 2 <= microsteps) m = static_cast<uint16_t>(m * 2);
    return m;
}

// MRES: 256 microsteps is 0, a full step 8.
int32_t microstepResolution(uint16_t microsteps) {
    int32_t mres = 8;
    for (uint32_t m = 2; m <= microsteps; m *= 2) mres--;
    return mres;
}

template <class T>
T clampTo(T v, T lo, T hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

constexpr uint32_t kGstatFlags = 0x07;  // reset, drv_err, uv_cp: written 1 to clear

}  // namespace

void TmcDriver::begin() {
    for (size_t i = 0; i < map_.registerCount; i++) {
        const TmcRegisterInfo& r = map_.registers[i];
        shadow_[r.address & 0x7F] = r.preset;
    }
    // The chip is read before the enable pin is touched: one that turns
    // through the ESP's reset is adopted, not stopped (and braked against
    // the rotor when enabled again).
    bool on = connect();
    if (enablePin_ >= 0) {
        {
            Guard g(lock());
            on = on && enabled_;
        }
        // DRV_ENN is active low: high disables.
        (gpio_ ? *gpio_ : defaultGpio()).output(enablePin_, !on);
        enableLevel_ = on ? 0 : 1;
    }
}

bool TmcDriver::identify() {
    uint32_t v = 0;
    busError_ = TmcError::Bus;
    if (!busRead(common_.version.reg, v)) {
        lastError_ = busError_;
        return false;
    }
    {
        Guard g(lock());
        shadow_[common_.version.reg & 0x7F] = v;
    }
    if (static_cast<uint32_t>(common_.version.get(v)) != common_.expectedVersion) {
        lastError_ = TmcError::WrongVersion;
        return false;
    }
    return true;
}

bool TmcDriver::connect() {
    if (!busBegin()) {
        errors_++;
        lastError_ = TmcError::Bus;
        return false;
    }
    if (!identify()) {
        errors_++;
        return false;
    }
    // What the chip holds in the registers nothing wrote: its status, its
    // position, and whether it turns.
    for (size_t i = 0; i < map_.registerCount; i++) {
        const TmcRegisterInfo& r = map_.registers[i];
        if (!r.readable()) continue;
        {
            Guard g(lock());
            if (isSet(dirty_, r.address)) continue;
        }
        uint32_t v = 0;
        busError_ = TmcError::Bus;
        if (!busRead(r.address, v)) {
            errors_++;
            lastError_ = busError_;
            return false;
        }
        Guard g(lock());
        shadow_[r.address & 0x7F] = v;
    }
    // A chip that kept its power (no reset flag) and turns.
    TmcFound found = TmcFound::Stopped;
    float fullSteps = 0;
    bool turning, on;
    {
        Guard g(lock());
        bool kept = get(common_.reset) == 0;
        bool moving = get(common_.stst) == 0 && get(common_.tstep) < 0xFFFFF;
        on = get(common_.toff) != 0 && get(common_.enn) == 0 && drives();
        turning = kept && moving;
    }
    if (turning) found = on && measure(fullSteps) ? TmcFound::Turning : TmcFound::Coasting;
    Write writes[kMaxWrites + 2], clears[kMaxClears];
    size_t n, clearCount;
    {
        Guard g(lock());
        configure(found, fullSteps);
        applySettings();
        n = take(writes, kMaxWrites, clears, clearCount);
        present_ = true;
    }
    flush(writes, n, clears, clearCount);
    found_ = true;
    reconnect_ = false;
    failures_ = 0;
    return true;
}

void TmcDriver::configure(TmcFound found, float fullStepsPerSecond) {
    uint16_t chip = chipMicrosteps();
    for (size_t i = 0; i < map_.registerCount; i++) {
        const TmcRegisterInfo& r = map_.registers[i];
        if (!r.configure) continue;
        // A register written since boot keeps what was written.
        if (!isSet(dirty_, r.address)) shadow_[r.address & 0x7F] = r.preset;
        mark(pending_, r.address);
    }
    for (size_t i = 0; i < 4; i++) pending_[i] |= dirty_[i];
    clearCount_ = 0;
    clears_[clearCount_++] = {common_.reset.reg, kGstatFlags};
    status_.found = found;
    if (found == TmcFound::Turning) {
        // It goes on at the velocity it had, until the deadman: the
        // application renews it, or it stops (on the ramp, if any).
        enabled_ = true;
        drive_ = Drive::Velocity;
        velocity_ = fullStepsPerSecond * static_cast<float>(applied_);
        velocityWritten_ = true;
        adopt(velocity_);
        if (chip != applied_) {
            // The resolution changes with it: the slower write first.
            ordered_[0] = applied_ < chip ? velocityRegister() : common_.mres.reg;
            ordered_[1] = applied_ < chip ? common_.mres.reg : velocityRegister();
            orderedCount_ = 2;
        }
        return;
    }
    // Otherwise it starts disabled, as at boot: a chip that was reset lost
    // its position.
    enabled_ = false;
    drive_ = Drive::Hold;
    velocity_ = 0;
}

uint16_t TmcDriver::chipMicrosteps() const {
    int32_t mres = get(common_.mres);
    return static_cast<uint16_t>(256u >> (mres > 8 ? 8 : mres));
}

void TmcDriver::applySettings() {
    const Settings& s = settings;
    applyResolution();
    set(common_.mres, microstepResolution(applied_));
    set(common_.intpol, s.interpolate);
    set(common_.shaft, s.invert);
    // TOFF 0 turns the power stage off.
    set(common_.toff, enabled_ ? clampTo<uint8_t>(chopper.toff, 1, 15) : 0);
    set(common_.hstrt, clampTo<uint8_t>(chopper.hstrt, 0, 7));
    set(common_.hend, clampTo<uint8_t>(chopper.hend, 0, 15));
    set(common_.tbl, clampTo<uint8_t>(chopper.tbl, 0, 3));
    // stealthChop runs while TSTEP, the clock cycles between 1/256
    // microsteps, is at least TPWMTHRS.
    int32_t threshold = 0;
    if (s.stealthChopMax > 0) {
        float t = static_cast<float>(clockHz_) / (256.0f * s.stealthChopMax);
        threshold = t >= 1048575.0f ? 1048575 : t < 1.0f ? 1 : static_cast<int32_t>(t);
    }
    set(common_.tpwmthrs, threshold);
    set(common_.pwmOfs, pwm.ofs);
    set(common_.pwmGrad, pwm.grad);
    set(common_.pwmFreq, clampTo<uint8_t>(pwm.freq, 0, 3));
    set(common_.pwmAutoscale, pwm.autoscale);
    set(common_.pwmAutograd, pwm.autograd);
    set(common_.freewheel, clampTo<uint8_t>(pwm.freewheel, 0, 3));
    set(common_.pwmReg, clampTo<uint8_t>(pwm.reg, 1, 15));
    set(common_.pwmLim, clampTo<uint8_t>(pwm.lim, 0, 15));
    applyChip();
    applyMotion();
}

void TmcDriver::applyResolution() {
    // A new resolution keeps the motor's speed: the velocities in microsteps
    // are scaled to it, and written with it.
    uint16_t want = powerOfTwo(settings.microsteps);
    if (want == applied_ || !canChangeResolution()) return;
    float factor = static_cast<float>(want) / static_cast<float>(applied_);
    velocity_ *= factor;
    rescale(factor);
    applied_ = want;
    set(common_.mres, microstepResolution(applied_));
    applyMotion();
    // Between the two writes the motor goes slower, never faster.
    ordered_[0] = factor > 1 ? common_.mres.reg : velocityRegister();
    ordered_[1] = factor > 1 ? velocityRegister() : common_.mres.reg;
    orderedCount_ = 2;
}

void TmcDriver::put(uint8_t address, uint32_t value) {
    address &= 0x7F;
    if (shadow_[address] == value) return;
    shadow_[address] = value;
    mark(pending_, address);
}

size_t TmcDriver::take(Write* out, size_t max, Write* clears, size_t& clearCount) {
    size_t n = 0;
    // The ordered ones first, on top of `max`.
    for (size_t i = 0; i < orderedCount_; i++) {
        uint8_t a = ordered_[i] & 0x7F;
        if (!isSet(pending_, a)) continue;
        unmark(pending_, a);
        mark(dirty_, a);
        out[n++] = {a, shadow_[a]};
    }
    orderedCount_ = 0;
    max += n;
    // Then in increasing addresses: a TMC5160's RAMPMODE before VMAX before
    // XTARGET.
    for (uint8_t a = 0; a < 128 && n < max; a++) {
        if (!isSet(pending_, a)) continue;
        unmark(pending_, a);
        mark(dirty_, a);
        out[n++] = {a, shadow_[a]};
    }
    clearCount = clearCount_;
    for (size_t i = 0; i < clearCount_; i++) clears[i] = clears_[i];
    clearCount_ = 0;
    return n;
}

void TmcDriver::flush(const Write* writes, size_t n, const Write* clears, size_t clearCount) {
    for (size_t i = 0; i < n; i++) {
        if (!busWrite(writes[i].address, writes[i].value)) {
            errors_++;
            mark(retry_, writes[i].address);
        }
    }
    for (size_t i = 0; i < clearCount; i++) {
        if (!busWrite(clears[i].address, clears[i].value)) errors_++;
    }
}

void TmcDriver::failed(uint64_t nowUs) {
    errors_++;
    if (++failures_ >= kAbsentAfter) {
        found_ = false;
        retryUs_ = nowUs + uint64_t(kRetryMs) * 1000;
    }
}

bool TmcDriver::read(uint8_t address, uint64_t nowUs) {
    uint32_t v = 0;
    busError_ = TmcError::Bus;
    if (!busRead(address, v)) {
        lastError_ = busError_;
        failed(nowUs);
        return false;
    }
    // A version that isn't the chip's is a garbled or floating bus.
    if (address == common_.version.reg && static_cast<uint32_t>(common_.version.get(v)) != common_.expectedVersion) {
        lastError_ = TmcError::WrongVersion;
        failed(nowUs);
        return false;
    }
    failures_ = 0;
    if (readCount_ < kMaxReads) reads_[readCount_++] = {address, v};
    return true;
}

void TmcDriver::sense(uint64_t nowUs) {
    readCount_ = 0;
    if (!found_) {
        if (retryUs_ == 0) {
            retryUs_ = nowUs + uint64_t(kRetryMs) * 1000;
        } else if (nowUs >= retryUs_) {
            retryUs_ = nowUs + uint64_t(kRetryMs) * 1000;
            connect();
        }
        return;
    }
    if (reconnect_) {
        if (!connect()) failed(nowUs);
        return;
    }
    Write writes[kMaxWrites + 2], clears[kMaxClears];
    size_t n, clearCount;
    bool on;
    {
        Guard g(lock());
        // What was asked since the last tick goes out in this one.
        applySettings();
        n = take(writes, writesPerTick() < kMaxWrites ? writesPerTick() : kMaxWrites, clears, clearCount);
        on = enabled_;
    }
    flush(writes, n, clears, clearCount);
    // DRV_ENN is active low.
    int level = on ? 0 : 1;
    if (enablePin_ >= 0 && level != enableLevel_) {
        (gpio_ ? *gpio_ : defaultGpio()).set(enablePin_, level != 0);
        enableLevel_ = level;
    }

    const uint8_t* fast = nullptr;
    size_t fastCount = fastRegisters(fast);
    for (size_t i = 0; i < fastCount; i++) {
        if (!read(fast[i], nowUs)) return;
    }
    // One other readable register in turn.
    for (size_t tries = 0; tries < map_.registerCount; tries++) {
        const TmcRegisterInfo& r = map_.registers[nextRead_];
        nextRead_ = (nextRead_ + 1) % map_.registerCount;
        bool isFast = false;
        for (size_t i = 0; i < fastCount; i++) isFast = isFast || fast[i] == r.address;
        if (!r.readable() || isFast) continue;
        read(r.address, nowUs);
        break;
    }
}

void TmcDriver::update(uint64_t nowUs) {
    Guard g(lock());
    present_ = found_;
    for (size_t i = 0; i < readCount_; i++) {
        uint8_t a = reads_[i].address & 0x7F;
        // A register about to be written keeps the value it will get.
        if (!isSet(pending_, a)) shadow_[a] = reads_[i].value;
        if (a == common_.reset.reg && common_.reset.get(reads_[i].value)) {
            status_.resets++;
            reconnect_ = true;
        }
    }
    readCount_ = 0;
    for (size_t i = 0; i < 4; i++) {
        pending_[i] |= retry_[i];
        retry_[i] = 0;
    }

    if (velocityWritten_) {
        velocityWritten_ = false;
        velocitySinceUs_ = nowUs;
    } else if (drive_ == Drive::Velocity && nowUs - velocitySinceUs_ >= uint64_t(settings.deadmanMs) * 1000) {
        drive_ = Drive::Hold;
        velocity_ = 0;
    }
    advance(lastUpdateUs_ ? static_cast<float>(nowUs - lastUpdateUs_) * 1e-6f : 0.0f);
    lastUpdateUs_ = nowUs;
    applySettings();

    Status& s = status_;
    s.errors = errors_;
    s.standstill = get(common_.stst) != 0;
    s.stealthChop = get(common_.stealth) != 0;
    s.current = static_cast<uint8_t>(get(common_.csActual));
    s.overtemperature = get(common_.ot) != 0;
    s.overtemperatureWarning = get(common_.otpw) != 0;
    s.shortToGround = get(common_.s2ga) || get(common_.s2gb);
    s.shortToSupply = get(common_.s2vsa) || get(common_.s2vsb);
    s.openLoad = get(common_.ola) || get(common_.olb);
    s.driverError = get(common_.drvErr) != 0;
    s.stallGuard = static_cast<uint16_t>(get(common_.sgResult));
    s.lastError = lastError_;
}

void TmcDriver::stop() {
    enabled_ = false;
    drive_ = Drive::Hold;
    velocity_ = 0;
}

void TmcDriver::enable(bool on) {
    Guard g(lock());
    if (on == enabled_) return;
    enabled_ = on;
    drive_ = Drive::Hold;
    velocity_ = 0;
}

bool TmcDriver::enabled() {
    Guard g(lock());
    return enabled_;
}

bool TmcDriver::setVelocity(float microstepsPerSecond) {
    Guard g(lock());
    if (!enabled_ || !present_ || !drives()) return false;
    if (!isfinite(microstepsPerSecond)) microstepsPerSecond = 0;
    velocity_ = microstepsPerSecond;
    drive_ = microstepsPerSecond != 0 ? Drive::Velocity : Drive::Hold;
    velocityWritten_ = true;
    return true;
}

float TmcDriver::velocity() {
    Guard g(lock());
    return drive_ == Drive::Velocity ? velocity_ : 0;
}

float TmcDriver::actualVelocity() {
    Guard g(lock());
    return measuredVelocity();
}

void TmcDriver::hold() {
    Guard g(lock());
    drive_ = Drive::Hold;
    velocity_ = 0;
}

void TmcDriver::overrideCurrent(float amps) {
    Guard g(lock());
    override_ = amps > 0 ? amps : 0;
}

float TmcDriver::currentOverride() {
    Guard g(lock());
    return override_;
}

void TmcDriver::claim(const char* owner) {
    Guard g(lock());
    owner_ = owner ? owner : "";
}

void TmcDriver::release() {
    Guard g(lock());
    owner_.clear();
}

std::string TmcDriver::owner() {
    Guard g(lock());
    return owner_;
}

void TmcDriver::setMicrosteps(uint16_t microsteps) {
    Guard g(lock());
    settings.microsteps = microsteps;
    applyResolution();
}

uint16_t TmcDriver::microsteps() {
    Guard g(lock());
    return applied_;
}

float TmcDriver::runAmps() const { return override_ > 0 ? override_ : settings.runCurrent; }

float TmcDriver::holdAmps() const {
    if (override_ > 0) return override_;
    return settings.holdCurrent < settings.runCurrent ? settings.holdCurrent : settings.runCurrent;
}

TmcDriver::Status TmcDriver::status() {
    Guard g(lock());
    Status s = status_;
    s.present = present_;
    return s;
}

bool TmcDriver::present() {
    Guard g(lock());
    return present_;
}

uint32_t TmcDriver::registerValue(uint8_t address) {
    Guard g(lock());
    return map_.find(address) ? shadow_[address & 0x7F] : 0;
}

bool TmcDriver::writeRegister(uint8_t address, uint32_t value) {
    const TmcRegisterInfo* r = map_.find(address);
    if (!r || !r->writable() || strcmp(r->name, "OTP_PROG") == 0) return false;
    Guard g(lock());
    if (r->access == TmcAccess::ReadClear) {
        if (clearCount_ == kMaxClears) return false;
        clears_[clearCount_++] = {r->address, value};
        return true;
    }
    shadow_[r->address & 0x7F] = value;
    mark(pending_, r->address);
    return true;
}

float TmcDriver::velocityUnits(float microstepsPerSecond) const {
    return microstepsPerSecond * 16777216.0f / static_cast<float>(clockHz_);
}

int32_t TmcDriver::currentScale(float amps, float ohms, float volts, float scale) {
    if (!(amps > 0)) return 0;
    float n = 32.0f * amps * 1.41421356f * ohms / (volts * scale);
    return clampTo<int32_t>(static_cast<int32_t>(lroundf(n)) - 1, 0, 31);
}

}  // namespace tessermotion
