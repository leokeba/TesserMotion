#pragma once

// The TMC5160 on SPI (docs/DESIGN.md section 4.7): a driver with its own ramp
// generator, which moves to positions and turns at velocities. Strapped for
// step and direction (SD_MODE high), it only drives: the steps come from
// elsewhere, and setTarget() and setVelocity() are refused.
//
// Positions count microsteps at the resolution in effect (XACTUAL's unit),
// from wherever the chip started. A change of microsteps is applied at
// standstill: the ramp generator's own velocity can't be rescaled.

#include <stdint.h>

#include "tessermotion/tmc_driver.h"

namespace tessermotion {

struct Tmc5160Config {
    int cs;
    int enable = -1;              // DRV_ENN, active low; -1: tied low on the board
    float rSense = 0.075f;        // Ω
    uint32_t clockHz = 12000000;  // the chip's clock: 12 MHz internal
    uint32_t spiHz = 4000000;
};

class Tmc5160 : public TmcDriver {
public:
    static constexpr uint8_t kVersion = 0x30;

    // The ramp generator's limits, for positioning and velocities.
    struct Ramp {
        float maxSpeed = 51200;      // microsteps/s (a turn/s of a 200-step motor at 256)
        float acceleration = 51200;  // microsteps/s², and deceleration
    };

    Tmc5160(Spi& bus, const Tmc5160Config& config, Gpio* gpio = nullptr);

    // Set before Motion starts, or under lock() afterwards (the tree does).
    Ramp ramp;

    // From any task: these take Motion's lock.
    // Moves to `microsteps` at maxSpeed. False unless enabled, present and
    // the ramp generator drives (not strapped for step and direction).
    bool setTarget(int32_t microsteps);
    // The target while positioning, else the position.
    int32_t target();
    int32_t position();
    // Strapped for step and direction: setTarget() and setVelocity() refused.
    bool stepDir();

    float maxCurrent() const override;

protected:
    bool busBegin() override;
    bool busRead(uint8_t address, uint32_t& value) override;
    bool busWrite(uint8_t address, uint32_t value) override;
    size_t fastRegisters(const uint8_t*& list) const override;
    size_t writesPerTick() const override { return 16; }
    void applyChip() override;
    void applyMotion() override;
    bool drives() const override;
    float measuredVelocity() const override;
    uint8_t velocityRegister() const override;
    // Only at standstill: the ramp generator's own velocity can't be
    // rescaled.
    bool canChangeResolution() const override;

private:
    bool transfer(uint8_t first, uint32_t data, uint32_t& reply);

    Spi& bus_;
    const Tmc5160Config config_;
    const SpiDevice device_;
    int32_t target_ = 0;  // under the lock
};

}  // namespace tessermotion
