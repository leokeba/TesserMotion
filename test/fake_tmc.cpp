#include "fake_tmc.h"

#include <string.h>

#include <string>

#include "tessermotion/tmc2209.h"

namespace fake {

namespace {

// The registers a chip answers reads of, besides write-only ones (which
// read 0 here).
bool readable5160(uint8_t a) {
    switch (a) {
        case 0x00: case 0x01: case 0x02: case 0x04: case 0x07: case 0x08: case 0x0C: case 0x12:
        case 0x20: case 0x21: case 0x22: case 0x2D: case 0x34: case 0x35: case 0x36: case 0x38:
        case 0x39: case 0x3B: case 0x3C: case 0x6A: case 0x6B: case 0x6C: case 0x6F: case 0x71:
        case 0x72: case 0x73:
            return true;
        default:
            return false;
    }
}

bool readable2209(uint8_t a) {
    switch (a) {
        case 0x00: case 0x01: case 0x02: case 0x05: case 0x06: case 0x07: case 0x12: case 0x41:
        case 0x6A: case 0x6B: case 0x6C: case 0x6F: case 0x70: case 0x71: case 0x72:
            return true;
        default:
            return false;
    }
}

}  // namespace

bool TmcChip::wrote(uint8_t address) const { return writeCount(address) > 0; }

size_t TmcChip::writeCount(uint8_t address) const {
    size_t n = 0;
    for (uint8_t a : writes) n += a == address;
    return n;
}

void TmcChip::write(uint8_t address, uint32_t value, bool readable) {
    address &= 0x7F;
    writes.push_back(address);
    written[address] = value;
    if (address == gstat) {
        regs[address] &= ~value;
    } else if (readable) {
        regs[address] = value;
    }
}

void Tmc5160::powerOn() {
    memset(regs, 0, sizeof regs);
    regs[0x00] = 0x00000008;  // GCONF: multistep_filt
    regs[0x01] = 0x01;        // GSTAT: reset
    regs[0x04] = (0x30u << 24) | (stepDir ? 0x40u : 0);
    regs[0x6C] = 0x10410150;  // CHOPCONF, TOFF 0
    regs[0x6F] = 0x80000000;  // DRV_STATUS: standstill
}

void Tmc5160::attach(Spi& bus, int cs) {
    bus.devices[cs] = [this](const uint8_t* tx, uint8_t* rx, size_t len) { transfer(tx, rx, len); };
}

void Tmc5160::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (!powered || len != 5) {
        memset(rx, 0xFF, len);
        return;
    }
    uint32_t reply = regs[requested_];
    uint8_t status = static_cast<uint8_t>((regs[0x01] & 0x03) | ((regs[0x6F] >> 28) & 0x08));
    rx[0] = status;
    rx[1] = static_cast<uint8_t>(reply >> 24);
    rx[2] = static_cast<uint8_t>(reply >> 16);
    rx[3] = static_cast<uint8_t>(reply >> 8);
    rx[4] = static_cast<uint8_t>(reply);
    uint8_t address = tx[0] & 0x7F;
    if (tx[0] & 0x80) {
        uint32_t v = (uint32_t(tx[1]) << 24) | (uint32_t(tx[2]) << 16) | (uint32_t(tx[3]) << 8) | tx[4];
        write(address, v, readable5160(address));
    }
    requested_ = address;
}

void Tmc2209::powerOn() {
    memset(regs, 0, sizeof regs);
    regs[0x00] = 0x00000101;  // GCONF: i_scale_analog, multistep_filt
    regs[0x01] = 0x01;        // GSTAT: reset
    regs[0x06] = 0x21u << 24;  // IOIN: version
    regs[0x6C] = 0x10000053;  // CHOPCONF
    regs[0x6F] = 0x80000000;  // DRV_STATUS: standstill
    regs[0x70] = 0xC10D0024;  // PWMCONF
}

void Tmc2209::attach(Uart& uart) {
    uart.onWrite = [this, &uart](const uint8_t* data, size_t len) { datagram(uart, data, len); };
}

void Tmc2209::datagram(Uart& uart, const uint8_t* data, size_t len) {
    if (echo) uart.send(std::string(reinterpret_cast<const char*>(data), len));
    if (!powered || len < 4 || data[0] != 0x05 || data[1] != address) return;
    if (data[len - 1] != tessermotion::Tmc2209::crc(data, len - 1)) {
        badCrc++;
        return;
    }
    uint8_t reg = data[2] & 0x7F;
    if (len == 8 && (data[2] & 0x80)) {
        uint32_t v = (uint32_t(data[3]) << 24) | (uint32_t(data[4]) << 16) | (uint32_t(data[5]) << 8) | data[6];
        write(reg, v, readable2209(reg));
        regs[0x02] = (regs[0x02] + 1) & 0xFF;  // IFCNT
    } else if (len == 4) {
        if (reg == 0x6A) regs[reg] = static_cast<uint32_t>(static_cast<int32_t>(regs[reg]) + mscntPerRead) & 1023u;
        uint32_t v = regs[reg];
        uint8_t r[8] = {0x05, 0xFF, reg, static_cast<uint8_t>(v >> 24), static_cast<uint8_t>(v >> 16),
                        static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v), 0};
        r[7] = tessermotion::Tmc2209::crc(r, 7);
        uart.send(std::string(reinterpret_cast<const char*>(r), sizeof r));
    }
}

}  // namespace fake
