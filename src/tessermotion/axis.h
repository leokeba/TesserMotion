#pragma once

// A closed-loop position servo on a circle (docs/DESIGN.md section 4.3): an
// angle sensor, a DC motor, and a PID between them.
//
// Internally everything is in the sensor's raw frame, in which a positive
// motor output must increase the angle (set the motor's `invert` for that).
// The output frame, position = ±raw + offset, is only how angles are shown
// and given, so changing the offset or the sensor's `invert` never moves the
// motor, and limits stay at the same mechanical stops.

#include <stdint.h>

#include <string>

#include "tessermotion/angle_sensor.h"
#include "tessermotion/dc_motor.h"
#include "tessermotion/motion.h"

namespace tessermotion {

enum class AxisState : uint8_t {
    Disabled,     // the motor is driven by hand (manual output), 0 by default
    Starting,     // enabled, waiting for a first reading
    Holding,      // at the target, within tolerance
    Moving,
    SensorFault,  // no reading for sensorTimeout: output 0
};

class Axis : public Part {
public:
    struct Tuning {
        float kp = 0.05f;
        float ki = 0;
        float kd = 0;
        float tolerance = 0.1f;         // °: no output inside
        float maxSpeed = 0;             // °/s for the setpoint; 0: it jumps to the target
        float integralLimit = 1;        // the integral term's limit, as a share of full output
        float derivativeFilter = 0.2f;  // weight of each new velocity sample (1: unfiltered)
        uint32_t sensorTimeoutMs = 200;
        float jogSpeed = 5;  // °/s
        // A manual output or a jog not renewed for this long stops: for a
        // client that never sends the 0 or the release.
        uint32_t deadmanMs = 1000;
    };
    struct SensorConfig {
        bool invert = false;  // the output frame turns the other way
        float offset = 0;     // °, added after invert
    };
    // An arc of the raw frame, from `begin` to `end` in increasing angles;
    // the axis never crosses the rest. begin == end: no limits.
    struct Limits {
        bool enabled = false;
        float begin = 0;
        float end = 0;
    };

    Axis(AngleSensor& sensor, DcMotor& motor) : sensor_(sensor), motor_(motor) {}

    // Settings: set them before Motion starts, or under lock() afterwards
    // (the tree does).
    Tuning tuning;
    SensorConfig sensorConfig;
    Limits limits;

    // From any task, a part's update() included: these take Motion's lock.
    void enable(bool on);
    bool enabled();
    // In the output frame, clamped to the limits; reached at maxSpeed. False
    // unless the axis is running (enabled, with a reading).
    bool setTarget(float degrees);
    // The same, without the slew: the setpoint jumps (sequencers, trackers).
    bool setTargetNow(float degrees);
    // An owner (a sequence, a tracker) drives the target: until release(),
    // the tree refuses writes to it with `busy`. Code isn't refused. `owner`
    // is a name for the message, such as "tracking".
    void claim(const char* owner);
    void release();
    // Empty when unclaimed.
    std::string owner();
    // Holds where the axis is: stops a slew.
    void hold();
    // Drives the motor by hand, -1..1, for the deadman time unless
    // rewritten. False unless the axis is disabled.
    bool setManualOutput(float u);
    // Moves the target at jogSpeed, towards increasing (direction > 0) or
    // decreasing output-frame angles, within the limits, for the deadman
    // time unless renewed; 0 stops, and so do setTarget(), hold(), a
    // disable or a sensor fault. False unless the axis is running.
    bool jog(int direction);
    float position();  // output frame
    float target();    // output frame
    float error();     // setpoint − position, along the allowed path
    float output();    // the motor's, -1..1
    float raw();       // the last reading
    uint32_t faults();  // failed reads since boot
    AxisState state();

    DcMotor& motor() { return motor_; }
    AngleSensor& sensor() { return sensor_; }

    // Part
    void begin() override;
    void sense(uint64_t nowUs) override;
    void update(uint64_t nowUs) override;
    void stop() override;

private:
    bool setTarget(float degrees, bool slew);
    void reset();
    float toOutput(float raw) const;
    float toRaw(float out) const;
    float clamp(float raw) const;
    float path(float from, float to) const;
    void drive(float u);

    AngleSensor& sensor_;
    DcMotor& motor_;

    // Sensed outside the lock, by the control task only.
    bool sampleOk_ = false;
    float sample_ = 0;

    // Under the lock. Angles in the raw frame.
    bool enabled_ = false;
    AxisState state_ = AxisState::Disabled;
    bool hasReading_ = false;
    uint64_t lastReadingUs_ = 0;
    uint32_t faults_ = 0;
    float raw_ = 0;
    float target_ = 0;
    float setpoint_ = 0;  // follows the target at maxSpeed
    float error_ = 0;
    float manual_ = 0;
    bool manualWritten_ = false;  // since the last tick
    uint64_t manualSinceUs_ = 0;
    int8_t jog_ = 0;  // in the raw frame: -1, 0, 1
    bool jogWritten_ = false;
    uint64_t jogSinceUs_ = 0;
    std::string owner_;
    // The PID's memory; lastUs_ 0: the next tick has no dt.
    uint64_t lastUs_ = 0;
    float lastRaw_ = 0;
    float integral_ = 0;
    float velocity_ = 0;
};

}  // namespace tessermotion
