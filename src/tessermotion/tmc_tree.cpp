#include <ctype.h>
#include <math.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tesser/status.h"
#include "tessermotion/tree_setting.h"
#include "tessermotion/trees.h"

namespace tessermotion {

namespace {

using tesser::Check;
using tesser::Object;
using tesser::Status;

// The register list's elements: indexes into a map, shared by the drivers
// of a chip. A deque, so adding a chip leaves the others' lists in place.
std::vector<uint8_t>& rows(const TmcMap& map) {
    static std::deque<std::pair<const TmcMap*, std::vector<uint8_t>>> all;
    for (auto& r : all)
        if (r.first == &map) return r.second;
    all.emplace_back(&map, std::vector<uint8_t>());
    std::vector<uint8_t>& v = all.back().second;
    for (size_t i = 0; i < map.registerCount; i++) v.push_back(static_cast<uint8_t>(i));
    return v;
}

const char* const kAccessNames[] = {"read", "write", "readWrite", "readClear"};

// Microsteps per full step, a power of 2 from 1 to 256: an enum whose value
// is the exponent, so the tree offers the choices the chip has.
enum class Microsteps : uint8_t { M1, M2, M4, M8, M16, M32, M64, M128, M256 };

Microsteps toChoice(uint16_t microsteps) {
    uint8_t e = 0;
    while (e < 8 && (2u << e) <= microsteps) e++;
    return static_cast<Microsteps>(e);
}

// "toff=3 hstrt=4 ...": a register's fields in its value.
std::string decode(const TmcMap& map, const TmcRegisterInfo& r, uint32_t value) {
    std::string s;
    for (uint8_t i = 0; i < r.fieldCount; i++) {
        const TmcFieldInfo& f = map.fields[r.firstField + i];
        TmcField field{r.address, f.shift, f.width, f.isSigned};
        if (!s.empty()) s += ' ';
        s += f.name;
        s += '=';
        s += std::to_string(field.get(value));
    }
    return s;
}

// The map's JSON in pieces: its head, then a register at a time, then its
// end.
std::string mapPiece(const TmcMap& map, size_t i) {
    std::string s;
    if (i == 0) {
        s += "{\"chip\":\"";
        s += map.chip;
        s += "\",\"source\":\"";
        s += map.source;
        s += "\",\"registers\":[";
    }
    if (i < map.registerCount) {
        const TmcRegisterInfo& r = map.registers[i];
        if (i > 0) s += ',';
        s += "{\"name\":\"";
        s += r.name;
        s += "\",\"address\":" + std::to_string(r.address);
        s += ",\"access\":\"";
        s += kAccessNames[static_cast<int>(r.access)];
        s += "\",\"fields\":[";
        for (uint8_t k = 0; k < r.fieldCount; k++) {
            const TmcFieldInfo& f = map.fields[r.firstField + k];
            if (k > 0) s += ',';
            s += "{\"name\":\"";
            s += f.name;
            s += "\",\"shift\":" + std::to_string(f.shift) + ",\"width\":" + std::to_string(f.width);
            s += f.isSigned ? ",\"signed\":true}" : ",\"signed\":false}";
        }
        s += "]}";
    }
    if (i + 1 >= map.registerCount) s += "]}";
    return s;
}

// Streams the map, a register at a time: the whole is several KB.
class MapSource : public tesser::FileSource {
public:
    explicit MapSource(const TmcMap& map) : map_(map) {}
    int read(uint8_t* buf, size_t cap) override {
        size_t n = 0;
        while (n < cap) {
            if (pos_ == piece_.size()) {
                if (next_ >= map_.registerCount) break;
                piece_ = mapPiece(map_, next_++);
                pos_ = 0;
            }
            size_t k = piece_.size() - pos_;
            if (k > cap - n) k = cap - n;
            piece_.copy(reinterpret_cast<char*>(buf) + n, k, pos_);
            pos_ += k;
            n += k;
        }
        return static_cast<int>(n);
    }

private:
    const TmcMap& map_;
    size_t next_ = 0;
    std::string piece_;
    size_t pos_ = 0;
};

// Refuses a tree write to a claimed driver, naming the owner. The message
// must outlive the call: requests run under the API lock, so one buffer
// serves every driver.
Check claimed(TmcDriver& d) {
    static std::string busy;
    std::string owner = d.owner();
    if (owner.empty()) return Check::ok();
    busy = "driven by " + owner;
    return Check::fail(Status::Busy, busy.c_str());
}

void describeControls(Object& o, TmcDriver& d) {
    o.value(
         "enabled", [&d] { return d.enabled(); },
         [&d](bool on) -> Check {
             Check c = claimed(d);
             if (!c.isOk()) return c;
             if (on && !d.present()) return Check::fail(Status::NotAllowed, "the driver doesn't answer");
             d.enable(on);
             return Check::ok();
         })
        .watch()
        .doc("The power stage: off, the motor turns freely and loses its position");
    o.value(
         "velocity", [&d] { return d.velocity(); },
         [&d](float v) -> Check {
             Check c = claimed(d);
             if (!c.isOk()) return c;
             if (d.setVelocity(v)) return Check::ok();
             return Check::fail(Status::NotAllowed,
                                d.enabled() ? "the steps come from the STEP input" : "the driver is disabled");
         })
        .range(-1000000, 1000000)
        .unit("µsteps/s")
        .watch()
        .ui("keepalive", 400)  // the client renews it well within the deadman (500 ms or more)
        .doc("Turns the motor, for the deadman time unless rewritten");
    o.action("stop",
             [&d]() -> Check {
                 Check c = claimed(d);
                 if (c.isOk()) d.hold();
                 return c;
             })
        .doc("Stops the motor, which holds where it stopped")
        .ui("icon", "stop");
    o.value("actualVelocity", [&d] { return d.actualVelocity(); })
        .unit("µsteps/s")
        .watch()
        .ui("widget", "chart")
        .doc("The velocity now: the TMC5160's, or the TMC2209's ramp");
}

void describeStatus(Object& o, TmcDriver& d) {
    auto& s = o.object("status");
    auto flag = [&d](bool TmcDriver::Status::*field) { return [&d, field] { return d.status().*field; }; };
    s.value("present", flag(&TmcDriver::Status::present)).watch().doc("The chip answers");
    s.value("standstill", flag(&TmcDriver::Status::standstill)).watch();
    s.value("stealthChop", flag(&TmcDriver::Status::stealthChop)).watch().doc("Running in stealthChop now");
    s.value("current", [&d] { return d.status().current; })
        .watch()
        .doc("The current scale, 0..31: (n + 1)/32 of full scale");
    s.value("overtemperature", flag(&TmcDriver::Status::overtemperature)).watch();
    s.value("overtemperatureWarning", flag(&TmcDriver::Status::overtemperatureWarning)).watch();
    s.value("shortToGround", flag(&TmcDriver::Status::shortToGround)).watch();
    s.value("shortToSupply", flag(&TmcDriver::Status::shortToSupply)).watch();
    s.value("openLoad", flag(&TmcDriver::Status::openLoad)).watch().doc("Unreliable at standstill and at speed");
    s.value("driverError", flag(&TmcDriver::Status::driverError)).watch().doc("The power stage shut down");
    s.value("stallGuard", [&d] { return d.status().stallGuard; })
        .watch()
        .doc("SG_RESULT: lower is more load (TMC2209: read in turn)");
    s.value("resets", [&d] { return d.status().resets; }).watch().doc("Times the chip lost power since boot");
    s.value("errors", [&d] { return d.status().errors; })
        .watch()
        .doc("Failed transfers since boot, searches for an absent chip included");
    s.value("lastError", [&d] { return d.status().lastError; },
            {"none", "bus", "noReply", "echoOnly", "badReply", "wrongVersion"})
        .watch()
        .doc("Why the last failed transfer failed");
    s.value("found", [&d] { return d.status().found; }, {"stopped", "turning", "coasting"})
        .watch()
        .doc("What the last search found: turning through the ESP's reset (adopted, still enabled), or coasting (wait before enabling)");
}

void describeSettings(Object& o, TmcDriver& d) {
    TmcDriver::Settings& s = d.settings;
    // To the hundredth below, in double: the schema's range is double.
    double maxCurrent = floor(static_cast<double>(d.maxCurrent()) * 100.0) / 100.0;
    auto& c = o.object("config").persist().label("Driver settings");
    setting(c, "runCurrent", d, s.runCurrent).range(0, maxCurrent).unit("A").step(0.01).doc("rms");
    setting(c, "holdCurrent", d, s.holdCurrent)
        .range(0, maxCurrent)
        .unit("A")
        .step(0.01)
        .doc("rms, at standstill; at most the run current");
    c.value(
         "microsteps",
         [&d, &s] {
             Guard g(d.lock());
             return toChoice(s.microsteps);
         },
         [&d](Microsteps m) { d.setMicrosteps(static_cast<uint16_t>(1u << static_cast<uint8_t>(m))); },
         {"1", "2", "4", "8", "16", "32", "64", "128", "256"})
        .ui("labels", "1 (full steps),2,4,8,16,32,64,128,256")
        .doc("Per full step. A change keeps the motor's speed: a TMC5160 applies it at standstill");
    setting(c, "interpolate", d, s.interpolate).doc("The chip interpolates to 256 microsteps");
    setting(c, "invert", d, s.invert).doc("Turns the other way");
    setting(c, "stealthChop", d, s.stealthChop).doc("The quiet chopper; off: spreadCycle");
    setting(c, "stealthChopMax", d, s.stealthChopMax)
        .range(0, 10000)
        .unit("steps/s")
        .ui("visibleWhen", "stealthChop=true")
        .doc("Full steps/s above which spreadCycle takes over; 0: never");
    setting(c, "deadman", d, s.deadmanMs).range(500, 10000).unit("ms").doc("A velocity not renewed for this long stops");

    TmcDriver::Chopper& ch = d.chopper;
    auto& chopper = o.object("chopper").persist().label("Chopper").ui("collapsed").doc("spreadCycle's timing, in the datasheet's units");
    setting(chopper, "toff", d, ch.toff).range(1, 15).doc("Off time; 1 only with tbl 2 or more");
    setting(chopper, "hstrt", d, ch.hstrt).range(0, 7).doc("Hysteresis start");
    setting(chopper, "hend", d, ch.hend).range(0, 15).doc("Hysteresis end: −3..12");
    setting(chopper, "tbl", d, ch.tbl).range(0, 3).doc("Blank time");

    TmcDriver::Pwm& p = d.pwm;
    auto& pwm = o.object("pwm").persist().label("stealthChop PWM").ui("collapsed").doc("PWMCONF, in the datasheet's units");
    setting(pwm, "ofs", d, p.ofs).doc("Amplitude offset: autoscale's start, at standstill");
    setting(pwm, "grad", d, p.grad).doc("Amplitude gradient with speed: autoscale's start");
    setting(pwm, "freq", d, p.freq).range(0, 3).doc("2/1024, 2/683, 2/512, 2/410 of the clock");
    setting(pwm, "autoscale", d, p.autoscale).doc("Regulates the amplitude to the current");
    setting(pwm, "autograd", d, p.autograd).doc("And the gradient");
    setting(pwm, "freewheel", d, p.freewheel)
        .range(0, 3)
        .doc("At standstill with a hold current of 0: normal, free, coils shorted low or high");
    setting(pwm, "reg", d, p.reg).range(1, 15).doc("autoscale's regulation loop gradient");
    setting(pwm, "lim", d, p.lim).range(0, 15).doc("autoscale's limit when switching from spreadCycle");
}

void describeRegisters(Object& o, TmcDriver& d) {
    const TmcMap& map = d.map();
    o.list("registers", rows(map),
           [&d](Object& e, uint8_t& i) {
               const TmcRegisterInfo& r = d.map().registers[i];
               e.value("name", [&r] { return std::string(r.name); });
               e.value("address", [&r] { return r.address; });
               e.value("access", [&r] { return r.access; }, {"read", "write", "readWrite", "readClear"});
               e.value("value", [&d, &r] { return d.registerValue(r.address); })
                   .doc("Last read, or written for a write-only register");
               e.value("fields", [&d, &r] { return decode(d.map(), r, d.registerValue(r.address)); });
               e.action("write",
                        [&d, &r](uint32_t value) -> Check {
                            if (!d.writeRegister(r.address, value)) return Check::fail(Status::NotAllowed, "not writable");
                            return Check::ok();
                        })
                   .doc("Writes the register; bits a setting or the motion owns follow them again on the next tick");
           })
        .key("name")
        .readOnly()
        .writeAccess(tesser::Access::Admin)
        .ui("collapsed")
        .label("Registers");
    o.file("map")
        .contentType("application/json")
        .download([&map](tesser::FileRequest& r) {
            r.filename = map.chip;
            for (char& c : r.filename) c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
            r.filename += ".json";
            return std::unique_ptr<tesser::FileSource>(new MapSource(map));
        })
        .doc("The registers and their fields, as JSON: for a register editor");
}

}  // namespace

std::string tmcMapJson(const TmcMap& map) {
    std::string s;
    for (size_t i = 0; i < map.registerCount; i++) s += mapPiece(map, i);
    return s;
}

void describe(Object& o, Tmc5160& d) {
    describeControls(o, d);
    o.value(
         "target", [&d] { return d.target(); },
         [&d](int32_t microsteps) -> Check {
             Check c = claimed(d);
             if (!c.isOk()) return c;
             if (d.setTarget(microsteps)) return Check::ok();
             return Check::fail(Status::NotAllowed,
                                d.enabled() ? "the steps come from the STEP input" : "the driver is disabled");
         })
        .unit("µsteps")
        .watch()
        .doc("Moves there at the ramp's speed");
    o.value("position", [&d] { return d.position(); }).unit("µsteps").watch().ui("widget", "chart");
    describeStatus(o, d);
    describeSettings(o, d);
    auto& ramp = o.object("ramp").persist().label("Ramp").ui("collapsed");
    setting(ramp, "maxSpeed", d, d.ramp.maxSpeed).range(1, 5000000).unit("µsteps/s");
    setting(ramp, "acceleration", d, d.ramp.acceleration).range(1, 4000000).unit("µsteps/s²").doc("And deceleration");
    describeRegisters(o, d);
}

void describe(Object& o, Tmc2209& d) {
    describeControls(o, d);
    describeStatus(o, d);
    describeSettings(o, d);
    auto& ramp = o.object("ramp").persist().label("Ramp").ui("collapsed");
    setting(ramp, "acceleration", d, d.ramp.acceleration)
        .range(0, 4000000)
        .unit("µsteps/s²")
        .doc("Up and down, also to a stop; 0: velocities jump");
    describeRegisters(o, d);
}

}  // namespace tessermotion
