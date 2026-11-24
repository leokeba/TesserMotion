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

const char* moveStateName(StepDirMotor::MoveState state) {
    switch (state) {
        case StepDirMotor::MoveState::Idle: return "idle";
        case StepDirMotor::MoveState::Running: return "running";
        case StepDirMotor::MoveState::Stopping: return "stopping";
        case StepDirMotor::MoveState::Complete: return "complete";
        case StepDirMotor::MoveState::Cancelled: return "cancelled";
        case StepDirMotor::MoveState::Interrupted: return "interrupted";
        case StepDirMotor::MoveState::Failed: return "failed";
    }
    return "failed";
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
        .doc("Ramps a velocity to a stop, or decelerates and drains a relative move; enabled")
        .ui("icon", "stop");
    o.action("moveRelative",
             [&m](JsonVariantConst arg) -> Check {
                 if (!arg.is<JsonObjectConst>()) {
                     return Check::fail(Status::InvalidValue, "expected {steps, maxSpeed, acceleration, deceleration}");
                 }
                 const JsonVariantConst steps = arg["steps"];
                 const JsonVariantConst maxSpeed = arg["maxSpeed"];
                 const JsonVariantConst acceleration = arg["acceleration"];
                 const JsonVariantConst deceleration = arg["deceleration"];
                 if (!steps.is<int32_t>() || !maxSpeed.is<float>() || !acceleration.is<float>() ||
                     !deceleration.is<float>()) {
                     return Check::fail(Status::InvalidValue, "steps must be an integer and speed/rates numbers");
                 }
                 Check c = claimed(m);
                 if (!c.isOk()) return c;
                 if (!m.moveRelative(steps.as<int32_t>(), maxSpeed.as<float>(), acceleration.as<float>(),
                                     deceleration.as<float>())) {
                     return Check::fail(Status::NotAllowed,
                                        "move refused: disabled, absent, busy, invalid parameters, or position unknown");
                 }
                 return Check::ok();
             })
        .doc("Moves an exact number of STEP pulses; maxSpeed is µsteps/s, acceleration and deceleration are µsteps/s²");
    o.value("commandedPosition256", [&m] { return m.commandedPosition256(); })
        .unit("1/256 full-step")
        .watch()
        .ui("widget", "chart")
        .doc("Open-loop endpoint in canonical 1/256 full-step units, independent of the chip's MRES");
    o.value("commandedPositionValid", [&m] { return m.commandedPositionValid(); })
        .watch()
        .doc("False after untracked velocity pulses, a mid-move MRES change, or an abrupt in-flight interruption");
    o.value("moveState", [&m] { return std::string(moveStateName(m.moveState())); })
        .watch()
        .doc("Relative move state: idle, running, stopping, complete, cancelled, interrupted, or failed");
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
