#pragma once

// Step and direction motors (docs/DESIGN.md section 4.9): a generator of step
// pulses for a driver strapped for step and direction (a TMC5160 with SD_MODE
// high, whose own motion is off), with a ramp, velocities and holds as a
// MotionDrive has. Several motors share one enable line (a StepDirGroup);
// each has its own step and direction pins and its own pulse output.
//
// A velocity is in µsteps of the chip's resolution (its MRES) per second: a
// STEP pulse is one µstep. The ramp changes the pulse period on every pulse:
// the period follows from the speed, and the speed from the acceleration and
// the period, so the output asks the source for periods in chunks and no
// interrupt runs per step. A velocity through zero ends the train at rest;
// the direction changes only then, after a setup time, before the first pulse
// of the new direction.
//
// The enable line is active low (DRV_ENN). Its power stage is the chip's
// (TOFF), set through the chip's TmcDriver: the group enables both.

#include <stdint.h>

#include <atomic>
#include <string>

#include "tessermotion/motion_drive.h"
#include "tessermotion/platform.h"
#include "tessermotion/tmc_driver.h"

namespace tessermotion {

class StepDirGroup;

// The ramp for one motor: the speed, the period of the next pulse, and the
// speed after it. Pure: the host tests and the platform's refill both use it.
class StepPlanner {
public:
    // The longest period (1 s): slower speeds still pulse once a second.
    static constexpr float kMaxPeriodS = 1.0f;

    // `acceleration` in µsteps/s² (0: speeds jump); `tickHz` the output's
    // clock; `minTicks` the shortest period.
    void configure(float acceleration, uint32_t tickHz, uint32_t minTicks);
    // Starts a train in direction `dir` (±1) from rest.
    void start(int dir);
    // The period of the next pulse, in ticks, toward `target` (signed µsteps/s;
    // a target against the train's direction brakes it to rest first). 0: the
    // train ends (at rest, with nothing to do).
    uint32_t next(int32_t target);
    // The speed magnitude, µsteps/s, and the train's direction.
    float speed() const { return speed_; }
    int dir() const { return dir_; }
    // Back at rest, without a period (after a stop).
    void rest() { speed_ = 0; }

private:
    float acceleration_ = 0;
    uint32_t tickHz_ = 1;
    uint32_t minTicks_ = 1;
    float speed_ = 0;
    int dir_ = 1;
};

struct StepDirConfig {
    int step;             // STEP
    int dir;              // DIR
    bool invert = false;  // a forward velocity drives DIR low
    uint32_t tickHz = 10000000;  // the output's clock: 0.1 µs
    uint32_t highTicks = 20;     // a pulse: 2 µs high at 10 MHz (the chip wants 100 ns)
};

class StepDirMotor : public MotionDrive, public Part, public PulseSource {
public:
    struct Settings {
        float maxSpeed = 51200;      // µsteps/s, the most a velocity can be
        uint32_t deadmanMs = 1000;   // a velocity not renewed for this long stops
        uint32_t dirSetupUs = 50;    // from DIR to the first pulse of the new direction
    };

    // `chip` is the driver strapped for step and direction: the power stage
    // (TOFF) and the resolution (MRES) are its. `group` holds the enable line.
    StepDirMotor(PulseOutput& output, TmcDriver& chip, StepDirGroup& group, const StepDirConfig& config,
                 Gpio* gpio = nullptr);

    // Set before Motion starts, or under lock() afterwards.
    Settings settings;

    // MotionDrive. enable() enables the whole group: the enable line is shared.
    void enable(bool on) override;
    bool enabled() override;
    bool setVelocity(float microstepsPerSecond) override;
    float velocity() override;
    float actualVelocity() override;
    void hold() override;
    uint16_t microsteps() override;
    void claim(const char* owner) override;
    void release() override;
    std::string owner() override;
    bool present() override;

    // Part
    void begin() override;
    void update(uint64_t now) override;
    // Disables the motor (through the group).
    void stop() override;

    // PulseSource, from the platform's refill.
    size_t nextPeriods(uint32_t* ticks, size_t max) override;

private:
    friend class StepDirGroup;
    // Under the lock: the train stops and the motor rests (the group disabled).
    void rest();

    PulseOutput& output_;
    TmcDriver& chip_;
    StepDirGroup& group_;
    const StepDirConfig config_;
    Gpio* gpio_;

    // Target and actual velocity: written under the lock, read by the refill.
    std::atomic<int32_t> target_{0};
    std::atomic<int32_t> actual_{0};
    // The refill's only.
    StepPlanner planner_;

    // Under the lock.
    float velocity_ = 0;       // as set
    uint64_t renewedUs_ = 0;   // when the velocity was last set
    int dir_ = 1;              // the DIR pin's direction
    uint64_t dirSetUs_ = 0;    // when DIR was set
    bool dirSet_ = false;
    std::string owner_;
};

// The enable line both motors share, and the motors it enables.
class StepDirGroup : public Part {
public:
    static constexpr size_t kMaxMotors = 4;

    // `enable`: EN, active low; -1: tied low on the board (the chips'
    // power stages then follow the enable calls alone).
    explicit StepDirGroup(int enable, Gpio* gpio = nullptr);

    // Adds a motor to the group, before begin().
    bool add(StepDirMotor& motor);

    // Enables or disables every motor's power stage and drives EN.
    void enable(bool on);
    bool enabled();

    // Part
    void begin() override;
    void update(uint64_t now) override { (void)now; }
    // Disables everything.
    void stop() override;

private:
    const int enable_;
    Gpio* gpio_;
    bool enabled_ = false;  // under the lock
    StepDirMotor* motors_[kMaxMotors] = {};
    size_t count_ = 0;
};

}  // namespace tessermotion
