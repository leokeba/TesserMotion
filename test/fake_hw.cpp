#include "fake_hw.h"

#include <string.h>

#include <utility>

namespace fake {

Gpio gpio;
Pwm pwm;
uint64_t now = 0;
Task task;

namespace {
uint32_t pendingDurationUs = 0;
}

bool I2c::read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    reads++;
    lastAddr = addr;
    lastReg = reg;
    lastLen = len;
    if (stuck) return false;
    if (failNext > 0) {
        failNext--;
        return false;
    }
    auto dev = regs.find(addr);
    if (dev == regs.end()) return false;  // NACK
    for (size_t i = 0; i < len; i++) buf[i] = dev->second[static_cast<uint8_t>(reg + i)];
    return true;
}

bool I2c::write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) {
    writes++;
    lastAddr = addr;
    lastReg = reg;
    lastLen = len;
    if (stuck) return false;
    if (failNext > 0) {
        failNext--;
        return false;
    }
    auto dev = regs.find(addr);
    if (dev == regs.end()) return false;
    for (size_t i = 0; i < len; i++) dev->second[static_cast<uint8_t>(reg + i)] = buf[i];
    return true;
}

size_t Uart::read(uint8_t* buf, size_t max) {
    reads++;
    size_t n = rx.size() - pos;
    if (n > max) n = max;
    if (n > chunk) n = chunk;
    memcpy(buf, rx.data() + pos, n);
    pos += n;
    return n;
}

void Task::tick(uint32_t durationUs) {
    now += periodUs;
    pendingDurationUs = durationUs;
    fn(now);
}

void reset() {
    gpio = Gpio();
    pwm = Pwm();
    now = 0;
    task = Task();
}

}  // namespace fake

namespace tessermotion {

Gpio& defaultGpio() { return fake::gpio; }
Pwm& defaultPwm() { return fake::pwm; }

// The tick's own duration shows as the clock moving while it runs.
uint64_t nowUs() {
    fake::now += fake::pendingDurationUs;
    fake::pendingDurationUs = 0;
    return fake::now;
}

bool startPeriodic(const TaskConfig& config, uint32_t periodUs, std::function<void(uint64_t)> tick) {
    if (!tick) return false;
    fake::task.config = config;
    fake::task.periodUs = periodUs;
    fake::task.fn = std::move(tick);
    return true;
}

}  // namespace tessermotion
