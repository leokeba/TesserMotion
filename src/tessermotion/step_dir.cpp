#include "tessermotion/step_dir.h"

#include <math.h>

#include "tessermotion/platform/lock.h"

namespace tessermotion {

namespace {

int signOf(int32_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

}  // namespace

void StepPlanner::configure(float acceleration, uint32_t tickHz, uint32_t minTicks) {
    tickHz_ = tickHz > 0 ? tickHz : 1;
    minTicks_ = minTicks > 0 ? minTicks : 1;
    if (!(acceleration > 0)) acceleration = 0;
    constexpr float kMaxAcceleration = 16777215.0f;
    if (acceleration > kMaxAcceleration) acceleration = kMaxAcceleration;
    accelerationQ8_ = static_cast<uint32_t>(acceleration * static_cast<float>(kSpeedScale) + 0.5f);
    if (acceleration > 0 && accelerationQ8_ == 0) accelerationQ8_ = 1;
    decelerationQ8_ = accelerationQ8_;
    maxSpeedQ8_ = 0;
    if (accelerationQ8_ == 0) {
        firstPeriodTicks_ = 1;
    } else {
        const float effectiveAcceleration = static_cast<float>(accelerationQ8_) / static_cast<float>(kSpeedScale);
        float period = sqrtf(2.0f / effectiveAcceleration);
        if (period > kMaxPeriodS) period = kMaxPeriodS;
        const float ticks = period * static_cast<float>(tickHz_);
        firstPeriodTicks_ =
            ticks >= 4294967295.0f ? UINT32_MAX : static_cast<uint32_t>(ticks + 0.5f);
    }
}

void StepPlanner::configureMove(float maxSpeed, float acceleration, float deceleration, uint32_t tickHz,
                                uint32_t minTicks) {
    configure(acceleration, tickHz, minTicks);
    if (!(deceleration > 0)) deceleration = 0;
    constexpr float kMaxAcceleration = 16777215.0f;
    if (deceleration > kMaxAcceleration) deceleration = kMaxAcceleration;
    decelerationQ8_ = static_cast<uint32_t>(deceleration * static_cast<float>(kSpeedScale) + 0.5f);
    if (deceleration > 0 && decelerationQ8_ == 0) decelerationQ8_ = 1;
    if (!(maxSpeed > 0)) maxSpeed = 0;
    if (maxSpeed > kMaxAcceleration) maxSpeed = kMaxAcceleration;
    maxSpeedQ8_ = static_cast<uint32_t>(maxSpeed * static_cast<float>(kSpeedScale) + 0.5f);
    if (maxSpeed > 0 && maxSpeedQ8_ == 0) maxSpeedQ8_ = 1;
    if (maxSpeedQ8_ > 0) {
        const uint64_t ticks =
            (static_cast<uint64_t>(tickHz_) * kSpeedScale + maxSpeedQ8_ - 1) / maxSpeedQ8_;
        if (ticks > firstPeriodTicks_) {
            firstPeriodTicks_ = ticks > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(ticks);
        }
    }
}

void StepPlanner::start(int dir) {
    speedQ8_ = 0;
    accelerationRemainder_ = 0;
    dir_ = dir > 0 ? 1 : -1;
}

uint32_t StepPlanner::next(int32_t target) {
    // The speed the train goes to: the target in its direction, else rest.
    const int64_t magnitude = target < 0 ? -static_cast<int64_t>(target) : static_cast<int64_t>(target);
    const uint64_t goal = signOf(target) == dir_ ? static_cast<uint64_t>(magnitude) * kSpeedScale : 0;
    if (speedQ8_ == 0 && goal == 0) return 0;
    return nextToward(goal, false);
}

uint32_t StepPlanner::nextMove(uint64_t stepsRemaining) {
    if (stepsRemaining == 0 || maxSpeedQ8_ == 0 || decelerationQ8_ == 0) return 0;
    const uint64_t distanceAfter = stepsRemaining - 1;
    const uint64_t maxSpeedSquared = static_cast<uint64_t>(maxSpeedQ8_) * maxSpeedQ8_;
    const uint64_t factor = static_cast<uint64_t>(decelerationQ8_) * kSpeedScale * 2;
    const uint64_t radicand = distanceAfter > maxSpeedSquared / factor ? maxSpeedSquared : distanceAfter * factor;
    const uint64_t goal = integerSqrt(radicand);
    const uint32_t period = nextToward(goal, true);
    if (stepsRemaining == 1) {
        speedQ8_ = 0;
        accelerationRemainder_ = 0;
    }
    return period;
}

uint32_t StepPlanner::nextStop() {
    if (speedQ8_ == 0 || decelerationQ8_ == 0) return 0;
    const uint64_t ticks = (static_cast<uint64_t>(tickHz_) * kSpeedScale + speedQ8_ / 2) / speedQ8_;
    uint32_t periodTicks = ticks > tickHz_ ? tickHz_ : static_cast<uint32_t>(ticks);
    if (periodTicks == 0) periodTicks = 1;
    const uint32_t out = periodTicks < minTicks_ ? minTicks_ : periodTicks;
    const uint64_t numerator = static_cast<uint64_t>(decelerationQ8_) * periodTicks + accelerationRemainder_;
    const uint64_t step = numerator / tickHz_;
    accelerationRemainder_ = numerator % tickHz_;
    speedQ8_ = step < speedQ8_ ? speedQ8_ - step : 0;
    return out;
}

uint32_t StepPlanner::nextToward(uint64_t goal, bool finiteMove) {
    if (speedQ8_ == 0 && accelerationQ8_ == 0) speedQ8_ = goal;
    if (speedQ8_ == 0 && goal == 0 && !finiteMove) return 0;

    // The period of this pulse: from the speed, or from rest, the first pulse
    // at a distance of one µstep under the acceleration (t = √(2/a)).
    uint32_t periodTicks = firstPeriodTicks_;
    if (speedQ8_ > 0) {
        const uint64_t ticks = (static_cast<uint64_t>(tickHz_) * kSpeedScale + speedQ8_ / 2) / speedQ8_;
        periodTicks = ticks > tickHz_ ? tickHz_ : static_cast<uint32_t>(ticks);
        if (periodTicks == 0) periodTicks = 1;
    }
    const uint32_t out = periodTicks < minTicks_ ? minTicks_ : periodTicks;

    // The speed after it: moved toward the goal by the acceleration over this
    // period, or jumped to it. A brake to rest keeps the train's direction
    // until it stops.
    if (accelerationQ8_ > 0 && speedQ8_ != goal) {
        const uint32_t acceleration = speedQ8_ < goal ? accelerationQ8_ : decelerationQ8_;
        const uint64_t numerator = static_cast<uint64_t>(acceleration) * periodTicks + accelerationRemainder_;
        const uint64_t step = numerator / tickHz_;
        accelerationRemainder_ = numerator % tickHz_;
        if (speedQ8_ < goal) {
            speedQ8_ = step < goal - speedQ8_ ? speedQ8_ + step : goal;
        } else if (speedQ8_ > goal) {
            speedQ8_ = step < speedQ8_ - goal ? speedQ8_ - step : goal;
        }
    } else if (accelerationQ8_ == 0) {
        speedQ8_ = goal;
    }
    return out;
}

uint64_t StepPlanner::integerSqrt(uint64_t value) {
    uint64_t result = 0;
    uint64_t bit = uint64_t(1) << 62;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

float StepPlanner::speed() const { return static_cast<float>(speedQ8_) / static_cast<float>(kSpeedScale); }

int32_t StepPlanner::roundedSpeed() const {
    const uint64_t rounded = (speedQ8_ + kSpeedScale / 2) / kSpeedScale;
    return rounded > static_cast<uint64_t>(INT32_MAX) ? INT32_MAX : static_cast<int32_t>(rounded);
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
    outputReady_.store(output_.begin(config_.step, config_.tickHz, config_.highTicks), std::memory_order_relaxed);
    gpio_->output(config_.dir, (dir_ > 0) != config_.invert);
    dirSet_ = false;
}

void StepDirMotor::update(uint64_t now) {
    Guard g(lock());
    if (moveActive_ && chip_.microsteps() != moveMicrosteps_) {
        if (moveOutputStarted_) output_.stop();
        interruptMove();
        planner_.rest();
        return;
    }
    if (moveActive_ && moveOutputStarted_ && !output_.running()) {
        finishMove();
        return;
    }
    // setVelocity() can sample the clock after Motion captured this tick's `now`.
    if (velocity_ != 0 && now >= renewedUs_ &&
        now - renewedUs_ > static_cast<uint64_t>(settings.deadmanMs) * 1000) {
        velocity_ = 0;
        target_.store(0, std::memory_order_relaxed);
    }
    if (!group_.enabled() || !outputReady_.load(std::memory_order_relaxed)) return;
    if (output_.running()) return;  // the train brakes to rest by itself

    // The train's acceleration, read when it starts (not while it plays).
    if (!moveActive_) planner_.configure(ramp.acceleration, config_.tickHz, config_.highTicks + 2);

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
    finiteMoveSource_.store(moveActive_, std::memory_order_relaxed);
    if (!output_.start(*this)) {
        finiteMoveSource_.store(false, std::memory_order_relaxed);
        startFailures_.fetch_add(1, std::memory_order_relaxed);
        if (moveActive_) {
            moveActive_ = false;
            moveOutputStarted_ = false;
            moveState_ = MoveState::Failed;
            moveStepsRequested_ = 0;
            cancelMoveRequested_.store(false, std::memory_order_relaxed);
            target_.store(0, std::memory_order_relaxed);
            planner_.rest();
        }
    } else if (moveActive_) {
        moveOutputStarted_ = true;
    }
}

void StepDirMotor::rest() {
    Guard g(lock());
    output_.stop();
    if (moveActive_) interruptMove();
    planner_.rest();
    velocity_ = 0;
    target_.store(0, std::memory_order_relaxed);
    actual_.store(0, std::memory_order_relaxed);
}

void StepDirMotor::interruptMove() {
    if (moveOutputStarted_) commandedPositionValid_ = false;
    finiteMoveSource_.store(false, std::memory_order_relaxed);
    cancelMoveRequested_.store(false, std::memory_order_relaxed);
    moveActive_ = false;
    moveOutputStarted_ = false;
    moveState_ = MoveState::Interrupted;
    moveDeltaPosition256_ = 0;
    moveStepsRequested_ = 0;
    moveStepsRemaining_ = 0;
    target_.store(0, std::memory_order_relaxed);
    actual_.store(0, std::memory_order_relaxed);
}

void StepDirMotor::finishMove() {
    finiteMoveSource_.store(false, std::memory_order_relaxed);
    const bool cancelled = cancelMoveRequested_.load(std::memory_order_relaxed);
    const uint32_t generated = moveStepsGenerated_.load(std::memory_order_relaxed);
    int64_t delta = moveDeltaPosition256_;
    if (cancelled) {
        const int64_t magnitude = static_cast<int64_t>(generated) * movePositionUnitsPerStep_;
        delta = moveDirection_ < 0 ? -magnitude : magnitude;
    } else if (generated != moveStepsRequested_) {
        commandedPositionValid_ = false;
        moveState_ = MoveState::Interrupted;
        moveActive_ = false;
        moveOutputStarted_ = false;
        moveDeltaPosition256_ = 0;
        moveStepsRequested_ = 0;
        moveStepsRemaining_ = 0;
        target_.store(0, std::memory_order_relaxed);
        actual_.store(0, std::memory_order_relaxed);
        planner_.rest();
        return;
    }
    if ((delta > 0 && commandedPosition256_ > INT64_MAX - delta) ||
        (delta < 0 && commandedPosition256_ < INT64_MIN - delta)) {
        commandedPositionValid_ = false;
        moveState_ = MoveState::Interrupted;
    } else {
        commandedPosition256_ += delta;
        moveState_ = cancelled ? MoveState::Cancelled : MoveState::Complete;
    }
    moveDeltaPosition256_ = 0;
    moveActive_ = false;
    moveOutputStarted_ = false;
    moveStepsRequested_ = 0;
    moveStepsRemaining_ = 0;
    cancelMoveRequested_.store(false, std::memory_order_relaxed);
    velocity_ = 0;
    target_.store(0, std::memory_order_relaxed);
    actual_.store(0, std::memory_order_relaxed);
    planner_.rest();
}

void StepDirMotor::enable(bool on) { group_.enable(on); }

bool StepDirMotor::enabled() { return group_.enabled(); }

bool StepDirMotor::setVelocity(float microstepsPerSecond) {
    Guard g(lock());
    if (moveActive_ || !group_.enabled() || !chip_.present()) return false;
    if (!isfinite(microstepsPerSecond)) microstepsPerSecond = 0;
    const float limit = settings.maxSpeed > 0 ? settings.maxSpeed : 0;
    if (microstepsPerSecond > limit) microstepsPerSecond = limit;
    if (microstepsPerSecond < -limit) microstepsPerSecond = -limit;
    velocity_ = microstepsPerSecond;
    target_.store(static_cast<int32_t>(lroundf(microstepsPerSecond)), std::memory_order_relaxed);
    if (microstepsPerSecond != 0) commandedPositionValid_ = false;
    renewedUs_ = nowUs();
    return true;
}

bool StepDirMotor::moveRelative(int32_t steps, float maxSpeed, float acceleration, float deceleration) {
    Guard g(lock());
    if (!group_.enabled() || !chip_.present() || !outputReady_.load(std::memory_order_relaxed) ||
        !commandedPositionValid_ || moveActive_ || velocity_ != 0 || output_.running()) {
        return false;
    }
    if (!isfinite(maxSpeed) || !isfinite(acceleration) || !isfinite(deceleration) || maxSpeed <= 0 ||
        acceleration <= 0 || deceleration <= 0 || acceleration > 4000000.0f || deceleration > 4000000.0f) {
        return false;
    }
    const float speedLimit = settings.maxSpeed > 0 && isfinite(settings.maxSpeed) ? settings.maxSpeed : 0;
    if (speedLimit <= 0) return false;
    if (maxSpeed > speedLimit) maxSpeed = speedLimit;
    if (maxSpeed > 51200.0f) maxSpeed = 51200.0f;
    const long roundedSpeed = lroundf(maxSpeed);
    if (roundedSpeed < 1) return false;

    const uint16_t microsteps = chip_.microsteps();
    if (microsteps == 0 || microsteps > kPositionUnitsPerFullStep ||
        kPositionUnitsPerFullStep % microsteps != 0) {
        return false;
    }
    const int64_t positionDelta = static_cast<int64_t>(steps) * (kPositionUnitsPerFullStep / microsteps);
    if ((positionDelta > 0 && commandedPosition256_ > INT64_MAX - positionDelta) ||
        (positionDelta < 0 && commandedPosition256_ < INT64_MIN - positionDelta)) {
        return false;
    }

    moveDeltaPosition256_ = positionDelta;
    moveMicrosteps_ = microsteps;
    movePositionUnitsPerStep_ = static_cast<uint32_t>(kPositionUnitsPerFullStep / microsteps);
    moveDirection_ = steps < 0 ? -1 : 1;
    moveStepsRemaining_ = steps < 0 ? static_cast<uint64_t>(-static_cast<int64_t>(steps))
                                    : static_cast<uint64_t>(steps);
    moveStepsRequested_ = static_cast<uint32_t>(moveStepsRemaining_);
    moveStepsGenerated_.store(0, std::memory_order_relaxed);
    cancelMoveRequested_.store(false, std::memory_order_relaxed);
    moveMaxSpeed_ = static_cast<float>(roundedSpeed);
    moveAcceleration_ = acceleration;
    moveDeceleration_ = deceleration;
    moveOutputStarted_ = false;
    velocity_ = 0;
    if (steps == 0) {
        moveActive_ = false;
        moveState_ = MoveState::Complete;
        moveDeltaPosition256_ = 0;
        moveStepsRequested_ = 0;
        moveStepsRemaining_ = 0;
        target_.store(0, std::memory_order_relaxed);
        return true;
    }
    moveActive_ = true;
    moveState_ = MoveState::Running;
    planner_.configureMove(moveMaxSpeed_, moveAcceleration_, moveDeceleration_, config_.tickHz,
                           config_.highTicks + 2);
    target_.store(signOf(steps) * static_cast<int32_t>(roundedSpeed), std::memory_order_relaxed);
    return true;
}

float StepDirMotor::velocity() {
    Guard g(lock());
    return velocity_;
}

float StepDirMotor::actualVelocity() { return static_cast<float>(actual_.load(std::memory_order_relaxed)); }

int64_t StepDirMotor::commandedPosition256() {
    Guard g(lock());
    return commandedPosition256_;
}

bool StepDirMotor::commandedPositionValid() {
    Guard g(lock());
    return commandedPositionValid_;
}

StepDirMotor::MoveState StepDirMotor::moveState() {
    Guard g(lock());
    return moveState_;
}

void StepDirMotor::hold() {
    Guard g(lock());
    if (moveActive_) {
        velocity_ = 0;
        target_.store(0, std::memory_order_relaxed);
        if (moveOutputStarted_) {
            cancelMoveRequested_.store(true, std::memory_order_relaxed);
            moveState_ = MoveState::Stopping;
        } else {
            finiteMoveSource_.store(false, std::memory_order_relaxed);
            moveActive_ = false;
            moveState_ = MoveState::Cancelled;
            moveDeltaPosition256_ = 0;
            moveStepsRequested_ = 0;
            moveStepsRemaining_ = 0;
            cancelMoveRequested_.store(false, std::memory_order_relaxed);
            planner_.rest();
            actual_.store(0, std::memory_order_relaxed);
        }
        return;
    }
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
    if (finiteMoveSource_.load(std::memory_order_relaxed)) {
        while (n < max) {
            const bool cancel = cancelMoveRequested_.load(std::memory_order_relaxed);
            const uint32_t period =
                cancel ? planner_.nextStop()
                       : (moveStepsRemaining_ > 0 ? planner_.nextMove(moveStepsRemaining_) : 0);
            if (period == 0) break;
            ticks[n++] = period;
            if (!cancel) --moveStepsRemaining_;
        }
        if (n > 0) moveStepsGenerated_.fetch_add(static_cast<uint32_t>(n), std::memory_order_relaxed);
    } else {
        const int32_t target = target_.load(std::memory_order_relaxed);
        while (n < max) {
            const uint32_t period = planner_.next(target);
            if (period == 0) break;
            ticks[n++] = period;
        }
    }
    const int32_t speed = planner_.roundedSpeed() * planner_.dir();
    actual_.store(speed, std::memory_order_relaxed);
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
