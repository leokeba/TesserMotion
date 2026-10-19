"""Generates the register maps of Trinamic drivers from TMC-API (docs/DESIGN.md
section 4.8).

    git clone https://github.com/analogdevicesinc/TMC-API /tmp/tmc-api
    uv run python tools/tmc_maps.py /tmp/tmc-api

writes src/tessermotion/tmc/<chip>_map.h for each chip in CHIPS: the register
addresses and fields as constants for the drivers, and the same as tables for
the tree's register list and map. The headers are committed; run this again
to move to a newer TMC-API, and review the diff.

TMC-API keeps a chip's registers in two headers: <IC>_HW_Abstraction.h (the
addresses, and each field as a mask, a shift, a register and a sign) and <IC>.h
(each register's access, a sample configuration, and the reset values of
write-only registers that have one).
"""

import os
import re
import subprocess
import sys

CHIPS = ["TMC2209", "TMC5160"]

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "tessermotion", "tmc")

# TMC-API's names that its own generator mangled, by chip: the datasheet's.
RENAMES = {
    "TMC5160": {
        "DIAG0_ERROR__ONLY_WITH_SD_MODE_1_": "diag0_error",
        "DIAG0_OTPW__ONLY_WITH_SD_MODE_1_": "diag0_otpw",
        "TFD_2__0_": "tfd",
        "TFD__": "fd3",
        "V1_": "v1",
    },
}

# Fields TMC-API gets wrong, by chip: (shift, width) from the datasheet. The
# generator refuses a mask that doesn't match its shift, so a new error shows.
FIXES = {
    "TMC2209": {
        # Masks 0x01, 0x02, 0x04 with shifts 0, 8, 16: the OTP bytes.
        "OTP0_BYTE_0_READ_DATA": (0, 8),
        "OTP1_BYTE_1_READ_DATA": (8, 8),
        "OTP2_BYTE_2_READ_DATA": (16, 8),
        # Mask 0x30 with shift 8: FACTORY_CONF bits 9..8.
        "OTTRIM": (8, 2),
    },
}

# Fields TMC-API doesn't give, by chip and register: (name, shift, width,
# signed) from the datasheet.
ADDS = {
    "TMC2209": {
        "TCOOLTHRS": [("tcoolthrs", 0, 20, False)],
        "SGTHRS": [("sgthrs", 0, 8, False)],
        "SG_RESULT": [("sg_result", 0, 10, False)],
    },
}

# Fields left out, by chip: TMC-API's second names for the same bits.
DROPS = {
    "TMC5160": {"TFD_ALL", "TFD_3"},  # tfd, fd3
}

CPP_KEYWORDS = {
    "and", "auto", "bool", "break", "case", "char", "class", "const", "default", "delete", "do",
    "double", "else", "enum", "float", "for", "goto", "if", "int", "long", "new", "not", "or",
    "register", "return", "short", "signed", "static", "switch", "this", "union", "unsigned",
    "void", "while", "xor",
}

# TMC-API's access codes.
ACCESS = {
    0x01: "Read",
    0x02: "Write",
    0x03: "ReadWrite",
    0x13: "ReadWrite",  # reads one register (IOIN), writes another (OUTPUT)
    0x23: "ReadClear",  # flags: writing 1 clears
    0x42: "Write",  # with a reset value (kRegisterConstants)
}


def fail(msg):
    sys.exit(f"tmc_maps.py: {msg}")


def number(text):
    text = text.strip().rstrip("uUlL")
    return int(text, 16) if text.lower().startswith("0x") else int(text)


def table(source, name):
    """The initializer of `static const ... <name>[...] = { ... };` as tokens."""
    m = re.search(re.escape(name) + r"\s*\[[^\]]*\]\s*=\s*\{(.*?)\};", source, re.S)
    if not m:
        fail(f"no table {name}")
    body = re.sub(r"//[^\n]*", "", m.group(1))
    return [t.strip() for t in body.split(",") if t.strip()]


def parse(root, chip):
    prefix = chip
    lower = chip.lower()
    ic = os.path.join(root, "tmc", "ic", chip)
    with open(os.path.join(ic, f"{chip}_HW_Abstraction.h")) as f:
        hw = f.read()
    with open(os.path.join(ic, f"{chip}.h")) as f:
        api = f.read()

    defines = {}
    for name, value in re.findall(r"^#define\s+" + prefix + r"_(\w+)\s+(0x[0-9A-Fa-f]+|\d+)\b", hw, re.M):
        defines.setdefault(name, number(value))

    # Each register's access, from its address.
    access = {}
    for address, token in enumerate(table(api, f"{lower}_registerAccess")):
        if token not in ("____", "0x00", "0"):
            access[address] = number(token)

    # Sample configuration (written at start) and reset values (not written).
    r_values = {m[0]: number(m[1]) for m in re.findall(r"^#define\s+(R[0-9A-F]{2})\s+(0x[0-9A-Fa-f]+)", api, re.M)}
    sample = {}
    for address, token in enumerate(table(api, f"{lower}_sampleRegisterPreset")):
        value = r_values.get(token)
        if value is None and token not in ("0", "N_A", "____"):
            value = number(token)
        if value:
            sample[address] = value
    constants = {}
    m = re.search(lower + r"_RegisterConstants\s*\[\s*\]\s*=\s*\{(.*?)\};", api, re.S)
    if m:
        for a, v in re.findall(r"\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*\}", m.group(1)):
            constants[number(a)] = number(v)

    # Fields, in the order TMC-API lists them.
    field_re = re.compile(
        r"^#define\s+" + prefix + r"_(\w+?)_FIELD\s+\(\(RegisterField\)\s*\{\s*"
        + prefix + r"_(\w+?)_MASK\s*,\s*" + prefix + r"_(\w+?)_SHIFT\s*,\s*" + prefix + r"_(\w+)\s*,\s*(true|false)\s*\}\)",
        re.M)
    renames = RENAMES.get(chip, {})
    fields = {}  # address → [(name, shift, width, signed)]
    names = {}  # address → register name, from the fields that name it
    seen = {}
    for raw, mask_name, shift_name, reg_name, signed in field_re.findall(hw):
        # The SPI status byte's bits, given as fields of registers they
        # aren't in.
        if raw.startswith("SPI_STATUS_") or raw in DROPS.get(chip, ()):
            continue
        if reg_name not in defines:
            # Spelled differently from its define (X_ENC for XENC).
            same = [n for n in defines if n.replace("_", "") == reg_name.replace("_", "")]
            if len(same) != 1:
                fail(f"{chip}: field {raw} names an unknown register {reg_name}")
            reg_name = same[0]
        address = defines[reg_name]
        mask = defines[mask_name + "_MASK"]
        shift = defines[shift_name + "_SHIFT"]
        width = bin(mask >> shift).count("1")
        if raw in FIXES.get(chip, {}):
            shift, width = FIXES[chip][raw]
        elif mask != ((1 << width) - 1) << shift:
            fail(f"{chip}: field {raw}'s mask {mask:#x} isn't contiguous")
        name = renames.get(raw, raw).lower()
        if name in CPP_KEYWORDS:
            name += "_"
        key = (address, name)
        if key in seen:
            if seen[key] != (shift, width):
                fail(f"{chip}: field {name} of {reg_name} defined twice, differently")
            continue  # TMC-API repeats a register's fields for each mode
        seen[key] = (shift, width)
        names.setdefault(address, reg_name)
        fields.setdefault(address, []).append((name, shift, width, signed == "true"))

    for reg_name, adds in ADDS.get(chip, {}).items():
        address = defines[reg_name]
        if fields.get(address):
            fail(f"{chip}: {reg_name} has fields now: drop it from ADDS")
        names.setdefault(address, reg_name)
        fields[address] = list(adds)

    # Registers TMC-API gives an access but no field (MSLUT words, X_COMPARE,
    # and so on): named by their address's define.
    for address in access:
        if address not in names:
            candidates = [n for n, v in defines.items() if v == address and not re.search(r"_(MASK|SHIFT)$", n)]
            candidates = [n for n in candidates if not n.startswith("SPI_STATUS") and n not in ("WRITE_BIT", "ADDRESS_MASK")]
            if len(candidates) != 1:
                fail(f"{chip}: register {address:#04x} has no single name ({candidates})")
            names[address] = candidates[0]

    # Unique field names across the chip, so each is one constant.
    by_name = {}
    for address, fs in fields.items():
        for f in fs:
            by_name.setdefault(f[0], []).append(address)
    for name, addresses in by_name.items():
        if len(addresses) > 1:
            for address in addresses:
                fields[address] = [
                    (f"{names[address].lower()}_{f[0]}" if f[0] == name else f[0], *f[1:]) for f in fields[address]
                ]

    registers = []
    for address in sorted(access):
        if address not in names:
            continue
        code = access[address]
        if code not in ACCESS:
            fail(f"{chip}: register {names[address]} has an unknown access {code:#x}")
        fs = fields.get(address, [])
        # A register split into a bit per field (the MSLUT words) is its
        # own value.
        if len(fs) >= 32 and all(f[2] == 1 for f in fs):
            fs = []
        if address in constants:
            preset, start = constants[address], False
        elif address in sample:
            preset, start = sample[address], True
        else:
            preset, start = 0, False
        registers.append((names[address], address, ACCESS[code], preset, start, fs))
    return registers


def revision(root):
    try:
        return subprocess.run(["git", "-C", root, "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def emit(chip, registers, source):
    ns = chip.lower()
    out = []
    w = out.append
    w(f"// Generated by tools/tmc_maps.py from {source} (tmc/ic/{chip}): don't edit.")
    w("// The registers and fields of the " + chip + " (docs/DESIGN.md section 4.8).")
    w("#pragma once")
    w("")
    w('#include "tessermotion/tmc_map.h"')
    w("")
    w("namespace tessermotion {")
    w(f"namespace {ns} {{")
    w("")
    w("// Register addresses.")
    for name, address, *_ in registers:
        w(f"constexpr uint8_t {name} = 0x{address:02X};")
    w("")
    w("// Fields.")
    for name, address, _, _, _, fs in registers:
        for fname, shift, width, signed in fs:
            w(f"constexpr TmcField {fname}{{0x{address:02X}, {shift}, {width}, {'true' if signed else 'false'}}};")
    w("")
    w("inline constexpr TmcFieldInfo kFields[] = {")
    for name, address, _, _, _, fs in registers:
        for fname, shift, width, signed in fs:
            w(f'    {{"{fname}", {shift}, {width}, {"true" if signed else "false"}}},')
    w("};")
    w("")
    w("inline constexpr TmcRegisterInfo kRegisters[] = {")
    first = 0
    for name, address, access, preset, start, fs in registers:
        w(f'    {{"{name}", 0x{address:02X}, TmcAccess::{access}, 0x{preset:08X}, {"true" if start else "false"}, '
          f"{first}, {len(fs)}}},")
        first += len(fs)
    w("};")
    w("")
    w(f'inline constexpr TmcMap kMap{{"{chip}", "{source}", kRegisters, sizeof kRegisters / sizeof kRegisters[0], '
      "kFields, sizeof kFields / sizeof kFields[0]};")
    w("")
    w(f"}}  // namespace {ns}")
    w("}  // namespace tessermotion")
    return "\n".join(out) + "\n"


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    root = sys.argv[1]
    source = "TMC-API " + revision(root)
    os.makedirs(OUT, exist_ok=True)
    for chip in CHIPS:
        registers = parse(root, chip)
        path = os.path.join(OUT, f"{chip.lower()}_map.h")
        with open(path, "w") as f:
            f.write(emit(chip, registers, source))
        n = sum(len(r[5]) for r in registers)
        print(f"wrote {os.path.relpath(path)}: {len(registers)} registers, {n} fields")


if __name__ == "__main__":
    main()
