#pragma once

// A Trinamic driver's registers and fields (docs/DESIGN.md section 4.8), as
// tools/tmc_maps.py generates them from TMC-API into tmc/<chip>_map.h: a
// constant per register and per field for the drivers, and the same as
// tables for the register list and the map in the tree.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace tessermotion {

enum class TmcAccess : uint8_t {
    Read,
    Write,
    ReadWrite,
    ReadClear,  // flags: reads them, and writing 1 clears them
};

// Bits shift..shift+width-1 of register `reg`.
struct TmcField {
    uint8_t reg;
    uint8_t shift;
    uint8_t width;
    bool isSigned;

    constexpr uint32_t mask() const { return (width >= 32 ? 0xFFFFFFFFu : (1u << width) - 1u) << shift; }
    // The field in a register's value, sign-extended when signed.
    constexpr int32_t get(uint32_t value) const {
        uint32_t v = (value & mask()) >> shift;
        if (isSigned && width < 32 && (v >> (width - 1)) & 1u) v |= ~((1u << width) - 1u);
        return static_cast<int32_t>(v);
    }
    // The register's value with the field set to `v` (truncated to the width).
    constexpr uint32_t set(uint32_t value, int32_t v) const {
        return (value & ~mask()) | ((static_cast<uint32_t>(v) << shift) & mask());
    }
};

struct TmcFieldInfo {
    const char* name;
    uint8_t shift;
    uint8_t width;
    bool isSigned;
};

struct TmcRegisterInfo {
    const char* name;
    uint8_t address;
    TmcAccess access;
    // TMC-API's sample configuration (written at start when `configure`),
    // or the reset value of a write-only register (not written), or 0.
    uint32_t preset;
    bool configure;
    uint16_t firstField;  // in TmcMap::fields
    uint8_t fieldCount;

    bool readable() const { return access != TmcAccess::Write; }
    bool writable() const { return access != TmcAccess::Read; }
};

struct TmcMap {
    const char* chip;
    const char* source;  // the TMC-API revision it came from
    const TmcRegisterInfo* registers;
    size_t registerCount;
    const TmcFieldInfo* fields;
    size_t fieldCount;

    const TmcRegisterInfo* find(uint8_t address) const {
        for (size_t i = 0; i < registerCount; i++)
            if (registers[i].address == address) return &registers[i];
        return nullptr;
    }
    const TmcRegisterInfo* find(const char* name) const {
        for (size_t i = 0; i < registerCount; i++)
            if (strcmp(registers[i].name, name) == 0) return &registers[i];
        return nullptr;
    }
};

}  // namespace tessermotion
