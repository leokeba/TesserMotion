#pragma once

// What turns a motor at a velocity, whatever turns it (docs/DESIGN.md section
// 4.9): a Trinamic chip with its own ramp (the TMC2209, through its VACTUAL),
// or a step and direction generator driving a chip strapped for step and
// direction (StepDirMotor). An application (a Motor of motophonique, say)
// takes a MotionDrive& and doesn't care which.
//
// Velocities are signed, in microsteps per second at the resolution the
// drive reports (microsteps()). The deadman applies to every drive: a
// velocity not renewed for the drive's deadman stops the motor.
//
// Each call takes the drive's own lock, from any task, as the drives' own do.

#include <stdint.h>

#include <string>

namespace tessermotion {

class MotionDrive {
public:
    // The acceleration every drive ramps velocity with, up and down.
    struct Ramp {
        float acceleration = 0;  // µsteps/s²; 0: velocities jump
    };
    Ramp ramp;

    virtual ~MotionDrive() = default;

    // The power stage. Drives boot disabled; enabling or disabling stops any
    // motion.
    virtual void enable(bool on) = 0;
    virtual bool enabled() = 0;
    // Turns at `microstepsPerSecond` (signed) until stopped, or until the
    // deadman, unless renewed. False unless enabled and present.
    virtual bool setVelocity(float microstepsPerSecond) = 0;
    // The velocity as set: 0 when not turning at a velocity.
    virtual float velocity() = 0;
    // The velocity now, µsteps/s.
    virtual float actualVelocity() = 0;
    // Ramps to 0 and stays enabled, at the hold current.
    virtual void hold() = 0;
    // The resolution velocities count in.
    virtual uint16_t microsteps() = 0;
    // An owner (an application's part) drives the motor: the tree refuses
    // writes until release(). Code isn't refused.
    virtual void claim(const char* owner) = 0;
    virtual void release() = 0;
    virtual std::string owner() = 0;
    // Whether the drive's hardware answers.
    virtual bool present() = 0;
};

}  // namespace tessermotion
