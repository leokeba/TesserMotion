// A simulated plant (docs/DESIGN.md section 11): a geared DC motor behind a
// bridge, turning the magnet of a 14-bit encoder. Part of
// tessermotion_testing, for applications' tests too.
//
// The motor is a DcMotor (so the output mapping is part of the loop) and the
// encoder an AngleSensor, so an Axis runs on one Plant:
//
//   Plant plant;  Axis axis(plant, plant);
//
// Speed follows the duty with a first-order lag, above a dead band that
// static friction makes: τ·ω′ = maxSpeed·max(0, duty − deadBand)/(1 − deadBand) − ω.
// Duty 0 brakes. A positive output increases the angle.
#pragma once

#include <stdint.h>

#include "tessermotion/angle_sensor.h"
#include "tessermotion/dc_motor.h"

struct PlantParams {
    float maxSpeed = 30;         // °/s at full duty
    float timeConstant = 0.05f;  // s
    float deadBand = 0.12f;      // duty below which the motor doesn't turn
    float noise = 0.01f;         // ° of encoder noise (uniform, ±)
    // The encoder's error at a true angle, such as an off-centre magnet's.
    float (*sensorError)(float angle) = nullptr;
};

class Plant : public tessermotion::DcMotor, public tessermotion::AngleSensor {
public:
    using Params = PlantParams;

    explicit Plant(float angle = 0, Params params = Params(), uint32_t seed = 1)
        : params_(params), angle_(angle), seed_(seed) {}

    bool begin() override { return true; }
    bool read(float& degrees) override;

    // Runs the motor for `seconds`, in 0.5 ms steps.
    void advance(float seconds);

    float angle() const;                       // 0..360, the truth
    float turned() const { return angle_; }    // unwrapped, since the start
    float speed() const { return speed_; }     // °/s
    float drivenDuty() const { return duty_; }
    bool drivenForward() const { return forward_; }

    bool failing = false;  // reads fail while set
    uint32_t reads = 0;

protected:
    void drive(bool forward, float duty) override {
        forward_ = forward;
        duty_ = duty;
    }

private:
    float noise();

    Params params_;
    float angle_;
    float speed_ = 0;
    bool forward_ = true;
    float duty_ = 0;
    uint32_t seed_;
};
