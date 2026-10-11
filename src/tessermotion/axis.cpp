#include "tessermotion/axis.h"

#include <math.h>

#include "tessermotion/angle.h"

namespace tessermotion {

void Axis::enable(bool on) {
    Guard g(lock());
    if (on == enabled_) return;
    enabled_ = on;
    state_ = on ? AxisState::Starting : AxisState::Disabled;
    manual_ = 0;
    reset();
}

bool Axis::enabled() {
    Guard g(lock());
    return enabled_;
}

bool Axis::setTarget(float degrees) { return setTarget(degrees, true); }
bool Axis::setTargetNow(float degrees) { return setTarget(degrees, false); }

bool Axis::setTarget(float degrees, bool slew) {
    Guard g(lock());
    if (!enabled_ || state_ == AxisState::Starting || !(degrees == degrees)) return false;
    target_ = clamp(toRaw(degrees));
    if (!slew) setpoint_ = target_;
    return true;
}

void Axis::claim(const char* owner) {
    Guard g(lock());
    owner_ = owner && *owner ? owner : "an owner";
}

void Axis::release() {
    Guard g(lock());
    owner_.clear();
}

std::string Axis::owner() {
    Guard g(lock());
    return owner_;
}

void Axis::hold() {
    Guard g(lock());
    if (hasReading_) target_ = setpoint_ = raw_;
}

bool Axis::setManualOutput(float u) {
    Guard g(lock());
    if (enabled_) return false;
    manual_ = u;
    return true;
}

float Axis::position() {
    Guard g(lock());
    return toOutput(raw_);
}

float Axis::target() {
    Guard g(lock());
    return toOutput(target_);
}

float Axis::error() {
    Guard g(lock());
    return sensorConfig.invert ? -error_ : error_;
}

float Axis::output() {
    Guard g(lock());
    return motor_.lastOutput();
}

float Axis::raw() {
    Guard g(lock());
    return raw_;
}

uint32_t Axis::faults() {
    Guard g(lock());
    return faults_;
}

AxisState Axis::state() {
    Guard g(lock());
    return state_;
}

void Axis::begin() {
    motor_.begin();
    sensor_.begin();
    drive(0);
}

void Axis::sense(uint64_t) { sampleOk_ = sensor_.read(sample_); }

void Axis::update(uint64_t nowUs) {
    if (sampleOk_) {
        raw_ = sample_;
        hasReading_ = true;
        lastReadingUs_ = nowUs;
    } else {
        faults_++;
    }

    if (!enabled_) {
        // The target follows the position, for display and the next enable.
        if (hasReading_) target_ = setpoint_ = raw_;
        drive(manual_);
        return;
    }
    if (state_ == AxisState::Starting) {
        if (!sampleOk_) return drive(0);
        target_ = setpoint_ = raw_;  // never jump on start
        reset();
    }
    if (nowUs - lastReadingUs_ > uint64_t(tuning.sensorTimeoutMs) * 1000) {
        state_ = AxisState::SensorFault;
        reset();
        return drive(0);
    }
    if (!sampleOk_) return;  // keep the output until the timeout

    float dt = lastUs_ ? static_cast<float>(nowUs - lastUs_) * 1e-6f : 0;
    lastUs_ = nowUs;

    // The setpoint follows the target along the allowed path.
    float toGo = path(setpoint_, target_);
    bool slewing = false;
    if (tuning.maxSpeed > 0) {
        float stepMax = tuning.maxSpeed * dt;
        float step = fminf(fmaxf(toGo, -stepMax), stepMax);
        slewing = step != toGo;
        setpoint_ = slewing ? wrap360(setpoint_ + step) : target_;
    } else {
        setpoint_ = target_;
    }

    float e = path(raw_, setpoint_);
    error_ = e;
    if (dt > 0) {
        // Derivative on measurement: no kick when the setpoint jumps.
        float v = wrap180(raw_ - lastRaw_) / dt;
        velocity_ += tuning.derivativeFilter * (v - velocity_);
    }
    lastRaw_ = raw_;

    float u = 0;
    if (fabsf(e) < tuning.tolerance) {
        integral_ = 0;
    } else {
        if (tuning.ki > 0) {
            float limit = tuning.integralLimit / tuning.ki;
            integral_ = fminf(fmaxf(integral_ + e * dt, -limit), limit);
        } else {
            integral_ = 0;
        }
        u = tuning.kp * e + tuning.ki * integral_ - tuning.kd * velocity_;
    }
    state_ = slewing || fabsf(e) >= tuning.tolerance ? AxisState::Moving : AxisState::Holding;
    drive(u);
}

void Axis::stop() { enable(false); }

void Axis::reset() {
    integral_ = 0;
    velocity_ = 0;
    lastUs_ = 0;
}

float Axis::toOutput(float raw) const { return wrap360((sensorConfig.invert ? -raw : raw) + sensorConfig.offset); }

float Axis::toRaw(float out) const {
    float r = out - sensorConfig.offset;
    return wrap360(sensorConfig.invert ? -r : r);
}

float Axis::clamp(float raw) const {
    Arc arc(limits.enabled, limits.begin, limits.end);
    if (!arc.active) return wrap360(raw);
    float d = fminf(fmaxf(wrap180(raw - arc.middle), -arc.half), arc.half);
    return wrap360(arc.middle + d);
}

float Axis::path(float from, float to) const {
    Arc arc(limits.enabled, limits.begin, limits.end);
    // Measured from the arc's middle, the way never goes through the
    // forbidden side, which is opposite the middle.
    if (arc.active) return wrap180(to - arc.middle) - wrap180(from - arc.middle);
    return wrap180(to - from);
}

void Axis::drive(float u) { motor_.output(fminf(fmaxf(u, -1.0f), 1.0f)); }

}  // namespace tessermotion
