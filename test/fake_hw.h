// The platform in memory (docs/DESIGN.md section 3.3), for host tests:
// GPIO, PWM and I2C fakes, a settable clock, and a control task that tests
// run by hand. Part of tessermotion_testing, for applications' tests too.
#pragma once

#include <stdint.h>

#include <functional>
#include <map>
#include <vector>

#include "tessermotion/platform.h"

namespace fake {

struct Gpio : tessermotion::Gpio {
    std::map<int, bool> level;
    std::map<int, bool> isOutput;
    uint32_t writes = 0;

    void output(int pin, bool l) override {
        isOutput[pin] = true;
        level[pin] = l;
        writes++;
    }
    void set(int pin, bool l) override {
        level[pin] = l;
        writes++;
    }
};

struct Pwm : tessermotion::Pwm {
    struct Channel {
        int pin;
        uint32_t hz;
        float duty;
    };
    std::vector<Channel> channels;
    int capacity = 8;

    int attach(int pin, uint32_t hz) override {
        if (static_cast<int>(channels.size()) >= capacity) return -1;
        channels.push_back({pin, hz, 0});
        return static_cast<int>(channels.size()) - 1;
    }
    void duty(int channel, float fraction) override { channels.at(static_cast<size_t>(channel)).duty = fraction; }
    // The duty on `pin`, or -1 if it has no channel.
    float dutyOn(int pin) const {
        for (const Channel& c : channels)
            if (c.pin == pin) return c.duty;
        return -1;
    }
};

// Devices are register files; failNext makes the next transfers fail.
struct I2c : tessermotion::I2c {
    std::map<uint8_t, std::map<uint8_t, uint8_t>> regs;  // address → register → value
    int failNext = 0;
    bool stuck = false;  // every transfer fails until recover()
    uint32_t begins = 0, reads = 0, writes = 0, recovers = 0;
    uint8_t lastAddr = 0, lastReg = 0;
    size_t lastLen = 0;

    bool begin() override {
        begins++;
        return true;
    }
    bool read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) override;
    bool write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) override;
    void recover() override {
        recovers++;
        stuck = false;
    }
};

// What defaultGpio() and defaultPwm() return on the host.
extern Gpio gpio;
extern Pwm pwm;

// tessermotion::nowUs().
extern uint64_t now;

// What startPeriodic() was given: a test calls tick() to run the task.
struct Task {
    tessermotion::TaskConfig config;
    uint32_t periodUs = 0;
    std::function<void(uint64_t)> fn;
    bool started() const { return static_cast<bool>(fn); }
    // Advances the clock by one period and runs a tick, which takes
    // `durationUs` of the clock.
    void tick(uint32_t durationUs = 0);
};
extern Task task;

// Back to a fresh platform.
void reset();

}  // namespace fake
