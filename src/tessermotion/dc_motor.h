#pragma once

// A brushed DC motor behind an H-bridge (docs/DESIGN.md section 4.1): the
// output mapping and its settings. A bridge driver derives from it and
// implements begin() and drive().

namespace tessermotion {

class DcMotor {
public:
    virtual ~DcMotor() = default;

    virtual bool begin() = 0;

    // -1..1, the sign is the direction. Below 0.001 the duty is 0, which
    // brakes on bridges with a slow decay (the TB6612).
    void output(float u);
    float lastOutput() const { return output_; }
    float duty() const { return duty_; }  // 0..1, as driven

    // Settings, read on each output().
    bool invert = false;      // swaps the direction
    float minOutput = 0.15f;  // the duty where the motor starts to move
    float shape = 1;          // duty = |u|^shape × (1 − minOutput) + minOutput

protected:
    // `duty` 0..1.
    virtual void drive(bool forward, float duty) = 0;

private:
    float output_ = 0;
    float duty_ = 0;
    bool forward_ = true;
};

}  // namespace tessermotion
