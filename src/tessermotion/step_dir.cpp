#include "tessermotion/step_dir.h"

#include <math.h>

#include "tessermotion/platform/lock.h"

namespace tessermotion {

namespace {

int signOf(int32_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

}  // namespace

void StepPlanner::configure(float acceleration, uint32_t tickHz, uint32_t minTicks) {
    acceleration_ = acceleration > 0 ? acceleration : 0;
    tickHz_ = tickHz > 0 ? tickHz : 1;
    minTicks_ = minTicks > 0 ? minTicks : 1;
}

void StepPlanner::start(int dir) {
    speed_ = 0;
    dir_ = dir > 0 ? 1 : -1;
}

uint32_t StepPlanner::next(int32_t target) {
    // The speed the train goes to: the target in its direction, else rest.
    const int64_t magnitude = target < 0 ? -static_cast<int64_t>(target) : static_cast<int64_t>(target);
    const float goal = signOf(target) == dir_ ? static_cast<float>(magnitude) : 0.0f;
    if (speed_ == 0 && goal == 0) return 0;
    // From rest, without an acceleration, the speed jumps to the goal first.
    if (speed_ == 0 && acceleration_ <= 0) speed_ = goal;

    // The period of this pulse: from the speed, or from rest, the first pulse
    // at a distance of one µstep under the acceleration (t = √(2/a)).
    float period = speed_ > 0 ? 1.0f / speed_ : sqrtf(2.0f / acceleration_);
    if (period > kMaxPeriodS) period = kMaxPeriodS;
    const float ticks = period * static_cast<float>(tickHz_);
    const uint32_t out = ticks < static_cast<float>(minTicks_) ? minTicks_ : static_cast<uint32_t>(ticks + 0.5f);

    // The speed after it: moved toward the goal by the acceleration over this
    // period, or jumped to it. A brake to rest keeps the train's direction
    // until it stops.
    if (acceleration_ > 0) {
        const float step = acceleration_ * period;
        if (speed_ < goal) {
            speed_ = speed_ + step < goal ? speed_ + step : goal;
        } else if (speed_ > goal) {
            speed_ = speed_ - step > goal ? speed_ - step : goal;
        }
    } else {
        speed_ = goal;
    }
    return out;
}

StepDirMotor::StepDirMotor(PulseOutput& output, TmcDriver& chip, StepDirGroup& group, const StepDirConfig& config,
                           Gpio* gpio)
    : output_(output),
      chip_(chip),
      group_(group),
      config_(config),
      gpio_(gpio ? gpio : &defaultGpio()) {}

void StepDirMotor::begin() {
    Guard g(lock());
    output_.begin(config_.step, config_.tickHz, config_.highTicks);
    gpio_->output(config_.dir, (dir_ > 0) != config_.invert);
    dirSet_ = false;
}

void StepDirMotor::update(uint64_t now) {
    Guard g(lock());
    if (velocity_ != 0 && now - renewedUs_ > static_cast<uint64_t>(settings.deadmanMs) * 1000) {
        velocity_ = 0;
        target_.store(0, std::memory_order_relaxed);
    }
    if (!group_.enabled()) return;
    if (output_.running()) return;  // the train brakes to rest by itself

    // The train's acceleration, read when it starts (not while it plays).
    planner_.configure(ramp.acceleration, config_.tickHz, config_.highTicks + 2);

    const int want = signOf(target_.load(std::memory_order_relaxed));
    if (want == 0) return;
    if (!dirSet_ || want != dir_) {
        // At rest: the direction changes now, and the first pulse waits for the
        // setup time, in a later tick.
        dir_ = want;
        gpio_->set(config_.dir, (dir_ > 0) != config_.invert);
        dirSet_ = true;
        dirSetUs_ = now;
        return;
    }
    if (now - dirSetUs_ < settings.dirSetupUs) return;
    planner_.start(dir_);
    output_.start(*this);
}

void StepDirMotor::rest() {
    Guard g(lock());
    output_.stop();
    planner_.rest();
    velocity_ = 0;
    target_.store(0, std::memory_order_relaxed);
    actual_.store(0, std::memory_order_relaxed);
}

void StepDirMotor::enable(bool on) { group_.enable(on); }

bool StepDirMotor::enabled() { return group_.enabled(); }

bool StepDirMotor::setVelocity(float microstepsPerSecond) {
    Guard g(lock());
    if (!group_.enabled() || !chip_.present()) return false;
    if (!isfinite(microstepsPerSecond)) microstepsPerSecond = 0;
    const float limit = settings.maxSpeed > 0 ? settings.maxSpeed : 0;
    if (microstepsPerSecond > limit) microstepsPerSecond = limit;
    if (microstepsPerSecond < -limit) microstepsPerSecond = -limit;
    velocity_ = microstepsPerSecond;
    target_.store(static_cast<int32_t>(lroundf(microstepsPerSecond)), std::memory_order_relaxed);
    renewedUs_ = nowUs();
    return true;
}

float StepDirMotor::velocity() {
    Guard g(lock());
    return velocity_;
}

float StepDirMotor::actualVelocity() { return static_cast<float>(actual_.load(std::memory_order_relaxed)); }

void StepDirMotor::hold() {
    Guard g(lock());
    velocity_ = 0;
    target_.store(0, std::memory_order_relaxed);
}

uint16_t StepDirMotor::microsteps() { return chip_.microsteps(); }

void StepDirMotor::claim(const char* owner) {
    Guard g(lock());
    owner_ = owner ? owner : "";
}

void StepDirMotor::release() {
    Guard g(lock());
    owner_.clear();
}

std::string StepDirMotor::owner() {
    Guard g(lock());
    return owner_;
}

bool StepDirMotor::present() { return chip_.present(); }

void StepDirMotor::stop() { group_.enable(false); }

size_t StepDirMotor::nextPeriods(uint32_t* ticks, size_t max) {
    size_t n = 0;
    const int32_t target = target_.load(std::memory_order_relaxed);
    while (n < max) {
        const uint32_t period = planner_.next(target);
        if (period == 0) break;
        ticks[n++] = period;
    }
    const float speed = planner_.speed() * static_cast<float>(planner_.dir());
    actual_.store(static_cast<int32_t>(speed >= 0 ? speed + 0.5f : speed - 0.5f), std::memory_order_relaxed);
    return n;
}

StepDirGroup::StepDirGroup(int enable, Gpio* gpio) : enable_(enable), gpio_(gpio ? gpio : &defaultGpio()) {}

bool StepDirGroup::add(StepDirMotor& motor) {
    if (count_ >= kMaxMotors) return false;
    motors_[count_++] = &motor;
    return true;
}

void StepDirGroup::begin() {
    Guard g(lock());
    if (enable_ >= 0) gpio_->output(enable_, !enabled_);  // disabled: high
}

void StepDirGroup::enable(bool on) {
    Guard g(lock());
    if (on == enabled_) return;
    enabled_ = on;
    if (enable_ >= 0) gpio_->set(enable_, !on);
    for (size_t i = 0; i < count_; i++) {
        if (!on) motors_[i]->rest();
        motors_[i]->chip_.enable(on);
    }
}

bool StepDirGroup::enabled() {
    Guard g(lock());
    return enabled_;
}

void StepDirGroup::stop() { enable(false); }

}  // namespace tessermotion
