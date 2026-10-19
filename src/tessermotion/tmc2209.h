#pragma once

// The TMC2209 on its single-wire UART (docs/DESIGN.md section 4.7): a driver
// for step and direction inputs, configured over the UART, which can also
// turn the motor at a velocity by itself (VACTUAL), without steps.
//
// Wiring: the ESP's TX through 1 kΩ to PDN_UART, its RX straight to it; the
// driver tells the request's echo from the reply, and works without one.
// Up to four share a UART, at the addresses their MS1 and MS2 pins set.
//
// VACTUAL has no ramp of its own: with an acceleration, the driver ramps it
// every tick, towards a velocity and back to 0 on hold() or the deadman;
// without, it jumps, which can stall a loaded motor.

#include <stdint.h>

#include "tessermotion/tmc_driver.h"

namespace tessermotion {

struct Tmc2209Config {
    uint8_t address = 0;          // 0..3, from MS1 and MS2
    int enable = -1;              // EN, active low; -1: tied low on the board
    float rSense = 0.11f;         // Ω
    uint32_t clockHz = 12000000;  // the chip's clock: 12 MHz internal
};

class Tmc2209 : public TmcDriver {
public:
    static constexpr uint8_t kVersion = 0x21;
    // A reply comes within 1.1 ms at 115200 baud. The UART rounds the wait up
    // to whole RTOS ticks, plus one: 20 ms at 100 Hz, only spent on a chip
    // that doesn't answer.
    static constexpr uint32_t kTimeoutMs = 10;

    struct Ramp {
        float acceleration = 0;  // µsteps/s², up and down; 0: jumps
    };

    explicit Tmc2209(Uart& uart, const Tmc2209Config& config = Tmc2209Config(), Gpio* gpio = nullptr);

    // Set before Motion starts, or under lock() afterwards (the tree does).
    Ramp ramp;

    float maxCurrent() const override;

    // The datasheet's CRC-8 of a datagram's first `len` bytes.
    static uint8_t crc(const uint8_t* data, size_t len);

protected:
    bool busBegin() override;
    bool busRead(uint8_t address, uint32_t& value) override;
    bool busWrite(uint8_t address, uint32_t value) override;
    size_t fastRegisters(const uint8_t*& list) const override;
    // A write takes 0.7 ms at 115200 baud.
    size_t writesPerTick() const override { return 4; }
    void applyChip() override;
    void applyMotion() override;
    void advance(float seconds) override;
    float measuredVelocity() const override;
    uint8_t velocityRegister() const override;
    void rescale(float factor) override;

private:
    void drain();
    float target() const;  // under the lock: where the ramp goes

    Uart& uart_;
    const Tmc2209Config config_;
    bool echo_ = false;  // the line sends our datagrams back; learned on each read
    float ramped_ = 0;   // µsteps/s, under the lock
};

}  // namespace tessermotion
