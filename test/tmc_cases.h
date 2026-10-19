// Trinamic drivers (docs/DESIGN.md sections 4.7 and 4.8): the generated
// register maps, the TMC5160 on a fake SPI bus, the TMC2209 on a fake UART,
// and their trees.
#pragma once

#include <math.h>

#include <string>

#include <ArduinoJson.h>

#include "TesserAPI.h"
#include "fake_tmc.h"
#include "sim.h"
#include "tessermotion/tmc/tmc2209_map.h"
#include "tessermotion/tmc/tmc5160_map.h"

using namespace tessermotion;

namespace {

namespace r51 = tessermotion::tmc5160;
namespace r22 = tessermotion::tmc2209;

constexpr int kCs = 5;
constexpr int kEnablePin = 4;
constexpr uint32_t kTickUs = 20000;

// A driver alone in a Motion, ticked by hand at 50 Hz.
template <class Driver>
struct TmcRig {
    fake::Gpio gpio;
    Motion motion;
    uint64_t now = 1000000;

    void tick(int n = 1) {
        for (int i = 0; i < n; i++) {
            now += kTickUs;
            fake::now = now;
            motion.step(now);
        }
    }
    void run(float seconds) { tick(static_cast<int>(seconds * 1e6f / kTickUs + 0.5f)); }
};

struct Rig5160 : TmcRig<Tmc5160> {
    fake::Spi bus;
    fake::Tmc5160 chip;
    Tmc5160 driver{bus, Tmc5160Config{kCs, kEnablePin}, &gpio};

    explicit Rig5160(bool attach = true, bool begin = true) {
        if (attach) chip.attach(bus, kCs);
        motion.add(driver);
        if (begin) motion.begin();
    }
    uint32_t written(TmcField f) const { return static_cast<uint32_t>(f.get(chip.written[f.reg])); }
};

struct Rig2209 : TmcRig<Tmc2209> {
    fake::Uart uart;
    fake::Tmc2209 chip;
    Tmc2209 driver;

    explicit Rig2209(bool echo = true, uint8_t chipAddress = 0, uint8_t driverAddress = 0)
        : driver(uart, config(driverAddress), &gpio) {
        chip.echo = echo;
        chip.address = chipAddress;
        chip.attach(uart);
        motion.add(driver);
        motion.begin();
    }
    static Tmc2209Config config(uint8_t address) {
        Tmc2209Config c;
        c.address = address;
        c.enable = kEnablePin;
        return c;
    }
    uint32_t written(TmcField f) const { return static_cast<uint32_t>(f.get(chip.written[f.reg])); }
};

}  // namespace

TEST(tmc_field) {
    TmcField sgt{0x6D, 16, 7, true};
    CHECK_EQ(sgt.mask(), 0x007F0000u);
    CHECK_EQ(sgt.get(0x00400000), -64);
    CHECK_EQ(sgt.get(0x003F0000), 63);
    CHECK_EQ(sgt.set(0xFFFFFFFF, -1), 0xFFFFFFFFu);
    CHECK_EQ(sgt.set(0, -2), 0x007E0000u);
    // Truncated to the width, other bits kept.
    TmcField toff{0x6C, 0, 4, false};
    CHECK_EQ(toff.set(0xFFFFFFF0, 0x13), 0xFFFFFFF3u);
    TmcField x{0x21, 0, 32, true};
    CHECK_EQ(x.mask(), 0xFFFFFFFFu);
    CHECK_EQ(x.get(0xFFFFFFFF), -1);
    CHECK_EQ(x.set(0, -5), 0xFFFFFFFBu);
}

TEST(tmc_maps) {
    // Datasheet positions, through TMC-API.
    CHECK_EQ(r51::CHOPCONF, 0x6C);
    CHECK(r51::toff.reg == 0x6C && r51::toff.shift == 0 && r51::toff.width == 4);
    CHECK(r51::irun.reg == 0x10 && r51::irun.shift == 8 && r51::irun.width == 5);
    CHECK(r51::sgt.isSigned && r51::sgt.width == 7);
    CHECK(r51::xactual.isSigned && r51::xactual.width == 32);
    CHECK(r22::vactual.isSigned && r22::vactual.width == 24);
    CHECK(r22::en_spreadcycle.reg == 0x00 && r22::en_spreadcycle.shift == 2);
    // TMC-API's errata, fixed by the generator.
    CHECK(r22::ottrim.shift == 8 && r22::ottrim.width == 2);
    CHECK(r22::otp1_byte_1_read_data.shift == 8 && r22::otp1_byte_1_read_data.width == 8);
    // The tables agree with the constants.
    const TmcMap& m = r51::kMap;
    CHECK_EQ(std::string(m.chip), "TMC5160");
    const TmcRegisterInfo* chop = m.find("CHOPCONF");
    CHECK(chop && chop->address == 0x6C && chop->access == TmcAccess::ReadWrite && chop->configure);
    CHECK(m.find(uint8_t{0x6C}) == chop);
    CHECK(m.find("DRV_STATUS")->access == TmcAccess::Read);
    CHECK(m.find("GSTAT")->access == TmcAccess::ReadClear);
    // The MSLUT words are their own value; their reset values come along.
    CHECK(m.find("MSLUT0")->fieldCount == 0 && m.find("MSLUT0")->preset == 0xAAAAB554 && !m.find("MSLUT0")->configure);
    size_t fields = 0;
    for (size_t i = 0; i < m.registerCount; i++) {
        CHECK_EQ(m.registers[i].firstField, fields);
        fields += m.registers[i].fieldCount;
    }
    CHECK_EQ(fields, m.fieldCount);
}

TEST(tmc5160_begin_configures) {
    Rig5160 r;
    CHECK(r.driver.present());
    CHECK(!r.driver.enabled());
    // TMC-API's sample configuration, with the settings' bits.
    CHECK_EQ(r.chip.written[r51::GCONF], 0x0000000Cu);  // multistep_filt, en_pwm_mode
    CHECK(r.chip.wrote(r51::SHORT_CONF) && r.chip.wrote(r51::DRV_CONF));
    CHECK_EQ(r.written(r51::toff), 0u);  // disabled
    CHECK_EQ(r.written(r51::mres), 0u);  // 256
    CHECK_EQ(r.written(r51::intpol), 1u);
    // 0.5 A through 0.075 Ω: the scaler takes it, the run current at CS 31.
    CHECK_EQ(r.written(r51::global_scaler), 42u);
    CHECK_EQ(r.written(r51::irun), 31u);
    CHECK_EQ(r.written(r51::ihold), 15u);
    CHECK_EQ(r.written(r51::iholddelay), 7u);
    // The ramp: AMAX = a·2^41/fCLK², a trapezoid; held in velocity mode.
    CHECK_EQ(r.written(r51::amax), 782u);
    CHECK_EQ(r.written(r51::dmax), 782u);
    CHECK_EQ(r.written(r51::d1), 782u);
    CHECK_EQ(r.written(r51::vstop), 10u);
    CHECK_EQ(r.written(r51::rampmode), 1u);
    CHECK_EQ(r.chip.regs[r51::VMAX], 0u);
    // The reset flag cleared; the driver's pin high.
    CHECK_EQ(r.chip.regs[r51::GSTAT], 0u);
    CHECK(r.gpio.level[kEnablePin]);
    CHECK(r.bus.last.mode == 3 && r.bus.last.hz == 4000000);
    // Nothing more to write while nothing changes.
    r.tick(3);
    size_t n = r.chip.writes.size();
    r.tick(10);
    CHECK_EQ(r.chip.writes.size(), n);
}

TEST(tmc5160_absent_then_found) {
    Rig5160 r(false);
    CHECK(!r.driver.present());
    CHECK(!r.driver.status().present);
    r.chip.attach(r.bus, kCs);
    r.run(1.0f);
    CHECK(!r.driver.present());
    CHECK(r.chip.writes.empty());
    r.run(1.1f);
    CHECK(r.driver.present());
    CHECK(r.chip.wrote(r51::GCONF));
    CHECK_EQ(r.chip.regs[r51::GSTAT], 0u);
    // Enabling an absent driver is refused through the tree, not here; a
    // velocity is refused while it's disabled.
    CHECK(!r.driver.setVelocity(100));
}

TEST(tmc5160_enable_and_velocity) {
    Rig5160 r;
    r.driver.enable(true);
    r.tick();
    CHECK_EQ(r.written(r51::toff), 3u);
    CHECK(!r.gpio.level[kEnablePin]);
    // v[µsteps/s] = VMAX · fCLK / 2^24.
    CHECK(r.driver.setVelocity(51200));
    r.tick();
    CHECK_EQ(r.written(r51::rampmode), 1u);
    CHECK_EQ(r.written(r51::vmax), 71583u);
    CHECK_EQ(r.driver.velocity(), 51200.0f);
    // Capped at the ramp's maxSpeed.
    CHECK(r.driver.setVelocity(-200000));
    r.tick();
    CHECK_EQ(r.written(r51::rampmode), 2u);
    CHECK_EQ(r.written(r51::vmax), 71583u);
    // Renewed, it goes on; not renewed for the deadman, it stops.
    r.run(0.8f);
    CHECK(r.driver.setVelocity(-200000));
    r.run(0.8f);
    CHECK_EQ(r.written(r51::vmax), 71583u);
    r.run(0.3f);
    CHECK_EQ(r.written(r51::vmax), 0u);
    CHECK_EQ(r.driver.velocity(), 0.0f);
    // Disabling stops it and turns the power stage off.
    CHECK(r.driver.setVelocity(1000));
    r.tick();
    r.driver.enable(false);
    r.tick();
    CHECK_EQ(r.written(r51::vmax), 0u);
    CHECK_EQ(r.written(r51::toff), 0u);
    CHECK(r.gpio.level[kEnablePin]);
    CHECK(!r.driver.setVelocity(1000));
}

TEST(tmc5160_positioning) {
    Rig5160 r;
    CHECK(!r.driver.setTarget(1000));
    r.driver.enable(true);
    r.tick();
    CHECK(r.driver.setTarget(1000));
    r.tick();
    CHECK_EQ(r.written(r51::rampmode), 0u);
    CHECK_EQ(r.written(r51::vmax), 71583u);
    CHECK_EQ(r.chip.written[r51::XTARGET], 1000u);
    // RAMPMODE, VMAX, then XTARGET.
    size_t mode = 0, vmax = 0, target = 0;
    for (size_t i = 0; i < r.chip.writes.size(); i++) {
        if (r.chip.writes[i] == r51::RAMPMODE) mode = i;
        if (r.chip.writes[i] == r51::VMAX) vmax = i;
        if (r.chip.writes[i] == r51::XTARGET) target = i;
    }
    CHECK(mode < vmax && vmax < target);
    CHECK_EQ(r.driver.target(), 1000);
    r.chip.regs[r51::XACTUAL] = static_cast<uint32_t>(-250);
    r.tick();
    CHECK_EQ(r.driver.position(), -250);
    // Stopping leaves positioning: velocity mode at 0, decelerating.
    r.driver.hold();
    r.tick();
    CHECK_EQ(r.written(r51::rampmode), 1u);
    CHECK_EQ(r.written(r51::vmax), 0u);
    CHECK_EQ(r.driver.target(), -250);
}

TEST(tmc5160_step_dir) {
    Rig5160 r(true, false);
    r.chip.stepDir = true;
    r.chip.powerOn();
    r.motion.begin();
    CHECK(r.driver.present());
    CHECK(r.driver.stepDir());
    r.driver.enable(true);
    r.tick();
    // It still drives: current and chopper.
    CHECK_EQ(r.written(r51::toff), 3u);
    CHECK(!r.driver.setTarget(100));
    CHECK(!r.driver.setVelocity(100));
}

TEST(tmc5160_reset_reconfigures) {
    Rig5160 r;
    r.driver.settings.microsteps = 16;
    r.driver.enable(true);
    r.tick(2);
    CHECK(r.driver.setVelocity(1000));
    r.chip.regs[r51::XACTUAL] = 5000;
    r.tick();
    // The motor supply goes and comes back: registers at reset, the flag
    // set.
    r.chip.powerOn();
    r.chip.writes.clear();
    r.tick(3);
    TmcDriver::Status s = r.driver.status();
    CHECK_EQ(s.resets, 1u);
    CHECK(!r.driver.enabled());
    CHECK_EQ(r.driver.velocity(), 0.0f);
    CHECK_EQ(r.chip.regs[r51::GSTAT], 0u);
    CHECK_EQ(r.chip.written[r51::GCONF], 0x0000000Cu);
    CHECK_EQ(r.written(r51::mres), 4u);
    CHECK_EQ(r.written(r51::toff), 0u);
    CHECK_EQ(r.written(r51::global_scaler), 42u);
    CHECK_EQ(r.driver.position(), 0);
    // Once.
    r.tick(20);
    CHECK_EQ(r.driver.status().resets, 1u);
}

TEST(tmc5160_power_loss) {
    Rig5160 r;
    r.chip.powered = false;
    r.tick(3);
    CHECK(!r.driver.present());
    CHECK(r.driver.status().errors >= 3);
    r.chip.powered = true;
    r.chip.powerOn();
    r.chip.writes.clear();
    r.run(2.1f);
    CHECK(r.driver.present());
    CHECK(r.chip.wrote(r51::GCONF));
    CHECK(r.chip.wrote(r51::IHOLD_IRUN));
}

TEST(tmc5160_register_writes) {
    Rig5160 r;
    CHECK(r.driver.writeRegister(r51::DRV_CONF, 0x00080200));
    CHECK(!r.driver.writeRegister(r51::DRV_STATUS, 1));
    CHECK(!r.driver.writeRegister(r51::OTP_PROG, 0xBD00));
    CHECK(!r.driver.writeRegister(0x7F, 1));
    r.tick();
    CHECK_EQ(r.chip.written[r51::DRV_CONF], 0x00080200u);
    CHECK_EQ(r.driver.registerValue(r51::DRV_CONF), 0x00080200u);
    // Bits nothing owns stay; owned ones follow their setting.
    uint32_t chop = r.driver.registerValue(r51::CHOPCONF);
    CHECK(r.driver.writeRegister(r51::CHOPCONF, r51::diss2g.set(r51::toff.set(chop, 7), 1)));
    r.tick(2);
    CHECK_EQ(r.written(r51::diss2g), 1u);
    CHECK_EQ(r.written(r51::toff), 0u);
    CHECK_EQ(r.driver.registerValue(r51::CHOPCONF), r.chip.regs[r51::CHOPCONF]);
    // Flags are cleared by writing 1s.
    r.chip.regs[r51::GSTAT] = 0x02;
    CHECK(r.driver.writeRegister(r51::GSTAT, 0x02));
    r.tick();
    CHECK_EQ(r.chip.regs[r51::GSTAT], 0u);
    // Kept across a reset.
    r.chip.powerOn();
    r.tick(3);
    CHECK_EQ(r.chip.written[r51::DRV_CONF], 0x00080200u);
    CHECK_EQ(r.written(r51::diss2g), 1u);
}

TEST(tmc5160_reads_in_turn) {
    Rig5160 r;
    r.chip.regs[r51::MSCNT] = 123;
    r.chip.regs[r51::DRV_STATUS] = r51::ot.set(r51::s2gb.set(r51::cs_actual.set(0, 20), 1), 1);
    // Every readable register within a round.
    r.tick(static_cast<int>(r51::kMap.registerCount));
    CHECK_EQ(r.driver.registerValue(r51::MSCNT), 123u);
    TmcDriver::Status s = r.driver.status();
    CHECK(s.overtemperature);
    CHECK(s.shortToGround);
    CHECK(!s.shortToSupply);
    CHECK(!s.standstill);
    CHECK_EQ(s.current, 20);
}

TEST(tmc5160_settings) {
    Rig5160 r;
    TmcDriver::Settings& s = r.driver.settings;
    s.microsteps = 16;
    s.invert = true;
    s.stealthChop = false;
    s.stealthChopMax = 100;
    s.runCurrent = 1.0f;
    s.holdCurrent = 2.0f;  // capped at the run current
    r.driver.chopper.tbl = 1;
    r.tick();
    CHECK_EQ(r.written(r51::mres), 4u);
    CHECK_EQ(r.written(r51::shaft), 1u);
    CHECK_EQ(r.written(r51::en_pwm_mode), 0u);
    CHECK_EQ(r.chip.written[r51::TPWMTHRS], 468u);  // 12 MHz / (256 · 100)
    CHECK_EQ(r.written(r51::global_scaler), 84u);
    CHECK_EQ(r.written(r51::irun), 31u);
    CHECK_EQ(r.written(r51::ihold), 31u);
    CHECK_EQ(r.written(r51::tbl), 1u);
    CHECK(fabsf(r.driver.maxCurrent() - 3.064f) < 0.01f);
    // stopAll() disables.
    r.driver.enable(true);
    r.tick();
    r.motion.stopAll();
    r.tick();
    CHECK(!r.driver.enabled());
    CHECK_EQ(r.written(r51::toff), 0u);
}

TEST(tmc2209_crc) {
    // Datagrams from TMC-API's CRC (tmc_CRC8, polynomial 7, reflected).
    const uint8_t read[] = {0x05, 0x00, 0x06};
    const uint8_t read3[] = {0x05, 0x03, 0x6F};
    const uint8_t write[] = {0x05, 0x00, 0x80, 0x00, 0x00, 0x01, 0xC0};
    const uint8_t reply[] = {0x05, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x40};
    CHECK_EQ(Tmc2209::crc(read, sizeof read), 0x6F);
    CHECK_EQ(Tmc2209::crc(read3, sizeof read3), 0x69);
    CHECK_EQ(Tmc2209::crc(write, sizeof write), 0xF6);
    CHECK_EQ(Tmc2209::crc(reply, sizeof reply), 0x4F);
}

TEST(tmc2209_begin_configures) {
    for (bool echo : {true, false}) {
        Rig2209 r(echo);
        CHECK(r.driver.present());
        CHECK_EQ(r.chip.badCrc, 0u);
        // The UART configures it, stealthChop on.
        CHECK_EQ(r.chip.written[r22::GCONF], 0x000001C0u);
        // 0.5 A through 0.11 + 0.02 Ω is under half of full scale: vsense.
        CHECK_EQ(r.written(r22::vsense), 1u);
        CHECK_EQ(r.written(r22::irun), 15u);
        CHECK_EQ(r.written(r22::ihold), 7u);
        CHECK_EQ(r.written(r22::mres), 4u);  // 16
        CHECK_EQ(r.written(r22::toff), 0u);
        CHECK_EQ(r.chip.regs[r22::GSTAT], 0u);
        // Writes counted by the chip, one per datagram.
        CHECK_EQ(r.chip.regs[r22::IFCNT], r.chip.writes.size() & 0xFF);
        r.tick(30);
        CHECK(r.driver.present());
        CHECK_EQ(r.driver.status().errors, 0u);
    }
}

TEST(tmc2209_velocity) {
    Rig2209 r;
    r.driver.enable(true);
    r.tick();
    CHECK_EQ(r.written(r22::toff), 3u);
    CHECK(r.driver.setVelocity(1000));
    r.tick();
    CHECK_EQ(static_cast<int32_t>(r.written(r22::vactual)), 1398);  // 1000 · 2^24 / 12 MHz
    CHECK(r.driver.setVelocity(-1000));
    r.tick();
    CHECK_EQ(r22::vactual.get(r.chip.written[r22::VACTUAL]), -1398);
    CHECK_EQ(r.chip.written[r22::VACTUAL], 0x00FFFA8Au);
    r.driver.hold();
    r.tick();
    CHECK_EQ(r.chip.written[r22::VACTUAL], 0u);
}

TEST(tmc2209_addresses) {
    Rig2209 same(true, 2, 2);
    CHECK(same.driver.present());
    Rig2209 other(true, 2, 1);
    CHECK(!other.driver.present());
    CHECK(other.chip.writes.empty());
    Rig2209 none(false, 3, 0);
    CHECK(!none.driver.present());
}

TEST(tmc2209_loses_and_finds) {
    Rig2209 r;
    r.chip.powered = false;
    r.tick(3);
    CHECK(!r.driver.present());
    r.chip.powered = true;
    r.chip.powerOn();
    r.run(2.1f);
    CHECK(r.driver.present());
    CHECK_EQ(r.chip.written[r22::GCONF], 0x000001C0u);
}

namespace {

struct TmcTreeRig : Rig5160 {
    fake::Uart uart;
    fake::Tmc2209 chip2209;
    Tmc2209 small{uart};
    tesser::Api api;

    TmcTreeRig() {
        chip2209.attach(uart);
        motion.add(small);
        small.begin();
        api.mount("stepper", driver);
        api.mount("small", small);
    }
    struct Reply {
        tesser::Status status;
        std::string body;
    };
    Reply request(tesser::Op op, const std::string& path, const std::string& body = {}, bool admin = true) {
        JsonDocument doc;
        if (!body.empty()) deserializeJson(doc, body);
        tesser::Request req;
        req.op = op;
        req.path = path;
        req.body = doc.as<JsonVariantConst>();
        req.client.authenticated = admin;
        tesser::StringReply reply;
        api.handle(req, reply);
        return {reply.status, reply.body};
    }
    Reply get(const std::string& path) { return request(tesser::Op::Get, path); }
    Reply set(const std::string& path, const std::string& body) { return request(tesser::Op::Set, path, body); }
};

}  // namespace

TEST(tree_tmc) {
    TmcTreeRig r;
    CHECK_EQ(tesser::declarationErrors(), 0);
    CHECK_EQ(r.get("/stepper/status/present").body, "true");
    // A velocity or a target needs the driver enabled.
    CHECK(r.set("/stepper/velocity", "1000").status == tesser::Status::NotAllowed);
    CHECK(r.set("/stepper/target", "1000").status == tesser::Status::NotAllowed);
    CHECK(r.set("/stepper/enabled", "true").status == tesser::Status::Ok);
    CHECK(r.set("/stepper/velocity", "1000").status == tesser::Status::Ok);
    r.tick();
    CHECK_EQ(r.get("/stepper/velocity").body, "1000");
    CHECK(r.set("/stepper/stop", "").status == tesser::Status::Ok);
    CHECK_EQ(r.get("/stepper/velocity").body, "0");
    // Settings.
    CHECK(r.set("/stepper/config/microsteps", "12").status == tesser::Status::InvalidValue);
    CHECK(r.set("/stepper/config/microsteps", "32").status == tesser::Status::Ok);
    CHECK(r.set("/stepper/config/runCurrent", "4").status == tesser::Status::InvalidValue);  // over 3.06 A
    CHECK(r.set("/stepper/chopper/toff", "5").status == tesser::Status::Ok);
    r.tick();
    CHECK_EQ(r.written(r51::mres), 3u);
    CHECK_EQ(r.written(r51::toff), 5u);
    // The registers, by name.
    CHECK_EQ(r.get("/stepper/registers/CHOPCONF/address").body, "108");
    CHECK_EQ(r.get("/stepper/registers/CHOPCONF/access").body, "\"readWrite\"");
    std::string fields = r.get("/stepper/registers/CHOPCONF/fields").body;
    CHECK(fields.find("toff=5 ") != std::string::npos);
    CHECK(fields.find("mres=3 ") != std::string::npos);
    CHECK(r.get("/stepper/registers/NOPE").status == tesser::Status::NotFound);
    // Writing one needs an admin, and a writable register.
    CHECK(r.request(tesser::Op::Set, "/stepper/registers/DRV_CONF/write", "524800", false).status ==
          tesser::Status::Unauthorized);
    CHECK(r.request(tesser::Op::Set, "/stepper/registers/DRV_CONF/write", "524800").status == tesser::Status::Ok);
    CHECK(r.request(tesser::Op::Set, "/stepper/registers/DRV_STATUS/write", "1").status ==
          tesser::Status::NotAllowed);
    CHECK(r.set("/stepper/registers/DRV_CONF/value", "1").status == tesser::Status::ReadOnly);
    r.tick();
    CHECK_EQ(r.chip.written[r51::DRV_CONF], 524800u);
    // The TMC2209's tree has a ramp, but no positioning.
    CHECK_EQ(r.get("/small/status/present").body, "true");
    CHECK_EQ(r.get("/small/ramp/acceleration").body, "0");
    CHECK(r.get("/small/ramp/maxSpeed").status == tesser::Status::NotFound);
    CHECK(r.get("/small/target").status == tesser::Status::NotFound);
    // Fields the generator adds where TMC-API has none.
    r.chip2209.regs[r22::SG_RESULT] = 123;
    r.tick(static_cast<int>(r22::kMap.registerCount));
    CHECK_EQ(r.get("/small/registers/SG_RESULT/fields").body, "\"sg_result=123\"");
    CHECK_EQ(r.get("/small/status/stallGuard").body, "123");
    CHECK_EQ(r.get("/small/registers/VACTUAL/access").body, "\"write\"");
}

TEST(tree_tmc_map) {
    TmcTreeRig r;
    // Downloaded in small chunks, it's the same JSON as tmcMapJson().
    tesser::Request req;
    req.client.authenticated = true;
    tesser::StringReply reply;
    tesser::FileTransfer t(r.api);
    CHECK(t.openDownload("/stepper/map", req.client, reply) == tesser::FileTransfer::Open::Ok);
    CHECK_EQ(t.filename(), "tmc5160.json");
    std::string body;
    uint8_t buf[7];
    for (int n; (n = t.read(buf, sizeof buf)) > 0;) body.append(reinterpret_cast<char*>(buf), static_cast<size_t>(n));
    CHECK_EQ(body, tmcMapJson(r51::kMap));
    JsonDocument doc;
    CHECK(!deserializeJson(doc, body));
    CHECK_EQ(doc["chip"].as<std::string>(), "TMC5160");
    JsonArray regs = doc["registers"];
    CHECK_EQ(regs.size(), r51::kMap.registerCount);
    JsonObject chop;
    for (JsonObject o : regs)
        if (o["name"] == "CHOPCONF") chop = o;
    CHECK_EQ(chop["address"].as<int>(), 0x6C);
    CHECK_EQ(chop["fields"][0]["name"].as<std::string>(), "toff");
    CHECK_EQ(chop["fields"][0]["width"].as<int>(), 4);
    JsonDocument small;
    CHECK(!deserializeJson(small, tmcMapJson(r22::kMap)));
    CHECK_EQ(small["registers"].size(), r22::kMap.registerCount);
}

TEST(tmc2209_ramp) {
    Rig2209 r;
    r.driver.ramp.acceleration = 10000;  // µsteps/s²: 200 a tick at 50 Hz
    r.driver.enable(true);
    r.tick();
    // Up a step a tick, renewed as an application renews it.
    float last = 0;
    for (int i = 0; i < 4; i++) {
        CHECK(r.driver.setVelocity(1000));
        r.tick();
        float v = r.driver.actualVelocity();
        CHECK(fabsf(v - last - 200) < 1);
        last = v;
    }
    CHECK_EQ(r22::vactual.get(r.chip.written[r22::VACTUAL]), 839);  // 600 µsteps/s, written a tick after
    for (int i = 0; i < 3; i++) {
        CHECK(r.driver.setVelocity(1000));
        r.tick();
    }
    CHECK_EQ(r.driver.actualVelocity(), 1000.0f);
    CHECK_EQ(r22::vactual.get(r.chip.written[r22::VACTUAL]), 1398);
    // Reversing goes through 0.
    CHECK(r.driver.setVelocity(-1000));
    r.tick(3);
    CHECK(fabsf(r.driver.actualVelocity() - 400) < 1);
    // hold() ramps down, and so does the deadman.
    r.driver.hold();
    r.tick();
    CHECK(fabsf(r.driver.actualVelocity() - 200) < 1);
    r.tick(2);
    CHECK_EQ(r.driver.actualVelocity(), 0.0f);
    CHECK(r.driver.setVelocity(1000));
    r.run(0.2f);
    CHECK_EQ(r.driver.actualVelocity(), 1000.0f);
    r.run(1.0f);
    CHECK_EQ(r.driver.actualVelocity(), 0.0f);
    CHECK_EQ(r.chip.written[r22::VACTUAL], 0u);
    // Disabling stops it at once.
    for (int i = 0; i < 10; i++) {
        CHECK(r.driver.setVelocity(1000));
        r.tick();
    }
    r.driver.enable(false);
    r.tick();
    CHECK_EQ(r.driver.actualVelocity(), 0.0f);
    r.tick();
    CHECK_EQ(r.chip.written[r22::VACTUAL], 0u);
}

TEST(tmc_pwm_settings) {
    // The defaults are the chip's reset values.
    Rig2209 small;
    CHECK_EQ(small.chip.written[r22::PWMCONF], 0xC10D0024u);
    Rig5160 big;
    CHECK(!big.chip.wrote(r51::PWMCONF));
    CHECK_EQ(big.driver.registerValue(r51::PWMCONF), 0xC40C001Eu);
    // Fluante's start.
    small.driver.pwm.ofs = 255;
    small.driver.pwm.grad = 14;
    small.driver.pwm.freewheel = 1;
    small.tick();
    CHECK_EQ(small.written(r22::pwm_ofs), 255u);
    CHECK_EQ(small.written(r22::pwm_grad), 14u);
    CHECK_EQ(small.written(r22::freewheel), 1u);
    CHECK_EQ(small.written(r22::pwm_lim), 12u);
    // Owned by the settings: a raw write of them doesn't last.
    CHECK(small.driver.writeRegister(r22::PWMCONF, r22::pwm_ofs.set(0xC10D0024, 7)));
    small.tick(2);
    CHECK_EQ(small.written(r22::pwm_ofs), 255u);
    big.driver.pwm.reg = 0;  // not allowed: 1 at least
    big.tick();
    CHECK_EQ(big.written(r51::pwm_reg), 1u);
}

TEST(tmc_current_override) {
    Rig5160 r;
    r.driver.overrideCurrent(0.1f);
    CHECK_EQ(r.driver.currentOverride(), 0.1f);
    r.tick();
    CHECK_EQ(r.written(r51::global_scaler), 32u);  // the least
    CHECK_EQ(r.written(r51::irun), 7u);             // 32 · 0.1·√2·0.075 / (0.325 · 32/256) = 8.35, − 1
    CHECK_EQ(r.written(r51::ihold), 7u);
    CHECK_EQ(r.driver.settings.runCurrent, 0.5f);
    r.driver.overrideCurrent(0);
    r.tick();
    CHECK_EQ(r.written(r51::global_scaler), 42u);
    CHECK_EQ(r.written(r51::irun), 31u);
    CHECK_EQ(r.written(r51::ihold), 15u);
}

TEST(tmc_errors) {
    // An absent TMC5160: MISO floats high.
    Rig5160 big(false);
    big.tick();  // the counts reach status() on a tick
    CHECK_EQ(big.driver.status().errors, 1u);
    CHECK(big.driver.status().lastError == TmcError::NoReply);
    big.run(2.1f);
    CHECK_EQ(big.driver.status().errors, 2u);  // a search every 2 s
    // A chip answering with another version.
    big.chip.regs[r51::INP_OUT] = 0x11000000;
    big.chip.attach(big.bus, kCs);
    big.run(2.1f);
    CHECK(!big.driver.present());
    CHECK(big.driver.status().lastError == TmcError::WrongVersion);
    // A TMC2209 at another address: the echo, no reply.
    Rig2209 echoOnly(true, 2, 0);
    echoOnly.tick();
    CHECK(echoOnly.driver.status().lastError == TmcError::EchoOnly);
    Rig2209 silent(false, 2, 0);
    silent.tick();
    CHECK(silent.driver.status().lastError == TmcError::NoReply);
    // A bad CRC.
    Rig2209 garbled;
    garbled.chip.powered = false;
    garbled.uart.onWrite = [&garbled](const uint8_t* data, size_t len) {
        garbled.uart.send(std::string(reinterpret_cast<const char*>(data), len));
        if (len == 4) garbled.uart.send(std::string("\x05\xFF\x6F\x00\x00\x00\x00\x00", 8));
    };
    garbled.tick();
    CHECK(garbled.driver.status().lastError == TmcError::BadReply);
    // The TMC5160's load, from DRV_STATUS.
    Rig5160 r;
    r.chip.regs[r51::DRV_STATUS] = r51::sg_result.set(0, 300);
    r.chip.regs[r51::VACTUAL] = 71583;
    r.tick();
    CHECK_EQ(r.driver.status().stallGuard, 300);
    CHECK(fabsf(r.driver.actualVelocity() - 51200) < 1);
    CHECK(r.driver.status().lastError == TmcError::None);
}

TEST(tree_tmc_claim) {
    TmcTreeRig r;
    CHECK(r.set("/small/enabled", "true").status == tesser::Status::Ok);
    r.small.claim("spinner");
    CHECK_EQ(r.small.owner(), "spinner");
    auto refused = [&r](const std::string& path, const std::string& body) {
        TmcTreeRig::Reply reply = r.set(path, body);
        CHECK(reply.status == tesser::Status::Busy);
        CHECK(reply.body.find("driven by spinner") != std::string::npos);
    };
    refused("/small/enabled", "false");
    refused("/small/velocity", "100");
    refused("/small/stop", "");
    // Code isn't refused; the readouts stay.
    CHECK(r.small.setVelocity(100));
    r.tick();
    CHECK_EQ(r.get("/small/velocity").body, "100");
    CHECK_EQ(r.get("/small/enabled").body, "true");
    r.driver.claim("tracker");
    CHECK(r.set("/stepper/target", "10").status == tesser::Status::Busy);
    r.small.release();
    CHECK(r.set("/small/stop", "").status == tesser::Status::Ok);
    CHECK_EQ(r.get("/small/velocity").body, "0");
}
