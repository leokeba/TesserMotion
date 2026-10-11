#pragma once

// The platform layer (docs/DESIGN.md section 3.3): what drivers need from the
// chip, as narrow interfaces. src/tessermotion/platform/esp/ implements them
// on ESP-IDF (which Arduino-ESP32 3.x ships); test/fake_hw.* in memory.

#include <stddef.h>
#include <stdint.h>

#include <functional>

namespace tessermotion {

class Gpio {
public:
    virtual ~Gpio() = default;
    // Makes `pin` an output at `level`.
    virtual void output(int pin, bool level) = 0;
    virtual void set(int pin, bool level) = 0;
};

class Pwm {
public:
    virtual ~Pwm() = default;
    // Starts a PWM output on `pin` at duty 0; returns its channel, or -1.
    virtual int attach(int pin, uint32_t frequencyHz) = 0;
    // `fraction` 0..1.
    virtual void duty(int channel, float fraction) = 0;
};

class I2c {
public:
    virtual ~I2c() = default;
    // Sets the bus up; every device on it calls it, so it may run again.
    virtual bool begin() = 0;
    // Register reads and writes on the device at 7-bit address `addr`. False
    // on a NACK, a bus error or a timeout.
    virtual bool read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) = 0;
    virtual bool write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) = 0;
    // Frees a bus a device holds low (clock pulses) and resets the controller.
    virtual void recover() = 0;
};

// The chip's GPIO and LEDC, used by drivers given none (tests give fakes).
Gpio& defaultGpio();
Pwm& defaultPwm();

// A periodic task: `tick(nowUs)` every `periodUs`, from the platform clock.
struct TaskConfig {
    const char* name = "motion";
    uint32_t stackBytes = 4096;
    int priority = -1;  // -1: configMAX_PRIORITIES - 2
    int core = -1;      // -1: core 1 on dual-core chips, else any
};
bool startPeriodic(const TaskConfig& config, uint32_t periodUs, std::function<void(uint64_t nowUs)> tick);
uint64_t nowUs();

}  // namespace tessermotion
