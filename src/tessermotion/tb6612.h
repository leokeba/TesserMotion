#pragma once

// The TB6612FNG dual H-bridge (docs/DESIGN.md section 4.1): two channels
// sharing one standby pin.
//
//   IN1 H, IN2 L: forward     IN1 L, IN2 H: reverse
//   PWM low between pulses: short brake (slow decay), so duty 0 brakes.

#include <stdint.h>

#include "tessermotion/dc_motor.h"
#include "tessermotion/platform.h"

namespace tessermotion {

class Tb6612Standby {
public:
    // `gpio` null: the chip's.
    explicit Tb6612Standby(int pin, Gpio* gpio = nullptr) : pin_(pin), gpio_(gpio) {}

    // Drives STBY high. Channels call it once their own pins are set.
    void begin();
    // Low: every channel on the chip coasts, whatever it drives.
    void enable(bool on);

private:
    int pin_;
    Gpio* gpio_;
    bool begun_ = false;
};

struct Tb6612Pins {
    int pwm;
    int in1;
    int in2;
    uint32_t frequencyHz = 15000;
};

class Tb6612Channel : public DcMotor {
public:
    // `gpio`, `pwm` null: the chip's.
    Tb6612Channel(Tb6612Standby& standby, const Tb6612Pins& pins, Gpio* gpio = nullptr, Pwm* pwm = nullptr)
        : standby_(standby), pins_(pins), gpio_(gpio), pwm_(pwm) {}

    bool begin() override;

protected:
    void drive(bool forward, float duty) override;

private:
    Tb6612Standby& standby_;
    Tb6612Pins pins_;
    Gpio* gpio_;
    Pwm* pwm_;
    int channel_ = -1;
};

}  // namespace tessermotion
