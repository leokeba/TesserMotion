// The step and direction motors' trees (docs/DESIGN.md sections 4.9 and 9).

#include <string>

#include "tesser/status.h"
#include "tessermotion/step_dir.h"
#include "tessermotion/tree_setting.h"
#include "tessermotion/trees.h"

namespace tessermotion {

namespace {

using tesser::Check;
using tesser::Object;
using tesser::Status;

// Refuses a tree write to a claimed motor, naming the owner (one buffer
// serves every motor: requests run under the API lock).
Check claimed(StepDirMotor& m) {
    static std::string busy;
    std::string owner = m.owner();
    if (owner.empty()) return Check::ok();
    busy = "driven by " + owner;
    return Check::fail(Status::Busy, busy.c_str());
}

}  // namespace

void describe(Object& o, StepDirGroup& group) {
    o.value(
         "enabled", [&group] { return group.enabled(); },
         [&group](bool on) -> Check {
             group.enable(on);
             return Check::ok();
         })
        .watch()
        .doc("The enable line: every motor of the group, its power stages too");
}

void describe(Object& o, StepDirMotor& m) {
    o.value(
         "enabled", [&m] { return m.enabled(); },
         [&m](bool on) -> Check {
             Check c = claimed(m);
             if (!c.isOk()) return c;
             if (on && !m.present()) return Check::fail(Status::NotAllowed, "the driver doesn't answer");
             m.enable(on);
             return Check::ok();
         })
        .watch()
        .doc("The enable line, shared by the group: enabling one motor enables them all");
    o.value(
         "velocity", [&m] { return m.velocity(); },
         [&m](float v) -> Check {
             Check c = claimed(m);
             if (!c.isOk()) return c;
             if (m.setVelocity(v)) return Check::ok();
             return Check::fail(Status::NotAllowed, m.enabled() ? "the driver doesn't answer" : "the motor is disabled");
         })
        .range(-51200, 51200)
        .unit("µsteps/s")
        .watch()
        .ui("keepalive", 400)
        .doc("Turns the motor, for the deadman time unless rewritten");
    o.action("stop",
             [&m]() -> Check {
                 Check c = claimed(m);
                 if (c.isOk()) m.hold();
                 return c;
             })
        .doc("Ramps to a stop, enabled")
        .ui("icon", "stop");
    o.value("actualVelocity", [&m] { return m.actualVelocity(); })
        .unit("µsteps/s")
        .watch()
        .ui("widget", "chart")
        .doc("The velocity now, from the pulses");
    o.value("pulseOutputReady", [&m] { return m.pulseOutputReady(); })
        .doc("Whether the pulse output initialized successfully");
    o.value("pulseStartFailures", [&m] { return m.pulseStartFailures(); })
        .doc("Pulse train start attempts rejected by the output");
    o.value("microsteps", [&m] { return static_cast<uint32_t>(m.microsteps()); })
        .doc("The resolution velocities count in: the chip's MRES, which is also its STEP's");
    o.value("present", [&m] { return m.present(); }).watch().doc("The chip answers");
    o.value("owner", [&m] { return m.owner(); }).doc("The application that drives the motor, if any");

    auto& settings = o.object("settings").persist().label("Settings").ui("collapsed");
    setting(settings, "maxSpeed", m, m.settings.maxSpeed).range(1, 51200).unit("µsteps/s");
    setting(settings, "deadmanMs", m, m.settings.deadmanMs).range(100, 60000).unit("ms");
    setting(settings, "dirSetupUs", m, m.settings.dirSetupUs).range(1, 100000).unit("µs");
    auto& ramp = o.object("ramp").persist().label("Ramp").ui("collapsed");
    setting(ramp, "acceleration", m, m.ramp.acceleration)
        .range(0, 4000000)
        .unit("µsteps/s²")
        .doc("Up and down, also to a stop; 0: velocities jump");
}

}  // namespace tessermotion
