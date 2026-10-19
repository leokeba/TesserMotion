// Trinamic drivers in memory, for host tests: register files behind a
// TMC5160's SPI datagrams and a TMC2209's UART datagrams. Part of
// tessermotion_testing.
#pragma once

#include <stdint.h>

#include <vector>

#include "fake_hw.h"

namespace fake {

// What both keep: the registers reads return, and what was written.
struct TmcChip {
    uint32_t regs[128] = {};     // read back (readable registers)
    uint32_t written[128] = {};  // the last value written to each register
    std::vector<uint8_t> writes;  // the addresses written, in order
    bool powered = true;          // off: no answer
    uint8_t gstat = 0x01;         // GSTAT's address, cleared by writing 1s

    // Comes back from a power cycle: registers at their reset values, the
    // reset flag set.
    virtual void powerOn() = 0;
    virtual ~TmcChip() = default;
    bool wrote(uint8_t address) const;
    // How many times `address` was written.
    size_t writeCount(uint8_t address) const;
    void write(uint8_t address, uint32_t value, bool readable);
};

struct Tmc5160 : TmcChip {
    bool stepDir = false;  // SD_MODE high
    Tmc5160() { powerOn(); }
    void powerOn() override;
    // Answers on `bus` at chip select `cs`.
    void attach(Spi& bus, int cs);
    void transfer(const uint8_t* tx, uint8_t* rx, size_t len);

private:
    uint8_t requested_ = 0;  // what the next reply carries
};

struct Tmc2209 : TmcChip {
    uint8_t address = 0;
    bool echo = true;  // TX and RX on one wire: the requests come back
    int32_t mscntPerRead = 0;  // turning: MSCNT moves by this before each read of it
    uint32_t usPerDatagram = 0;  // the clock moves by this for each datagram (115200 baud: about 1100)
    uint32_t badCrc = 0;
    Tmc2209() { powerOn(); }
    void powerOn() override;
    // Answers on `uart`.
    void attach(Uart& uart);
    void datagram(Uart& uart, const uint8_t* data, size_t len);
};

}  // namespace fake
