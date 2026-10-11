#include <string>

#include "tesser/status.h"
#include "tesser/value_types.h"
#include "tessermotion/trees.h"

namespace tessermotion {

namespace {

using tesser::Check;
using tesser::Object;
using tesser::Status;
using tesser::ValueNode;

// A setting read and written under the part's lock, since the control task
// reads it while transports write it.
template <class T>
ValueNode& setting(Object& o, const char* name, Part& part, T& field) {
    return o.value(
        name,
        [&part, &field] {
            Guard g(part.lock());
            return field;
        },
        [&part, &field](T v) {
            Guard g(part.lock());
            field = v;
        });
}

}  // namespace

void describe(Object& o, Axis& a) {
    // Controls first, then the readouts. Only position and output are charted.
    o.value("state", [&a] { return a.state(); }, {"disabled", "starting", "holding", "moving", "sensorFault"}).watch();
    // Watched: whatever the control loop or code changes (stopAll(), the
    // manual output's timeout) reaches subscribers.
    o.value("enabled", [&a] { return a.enabled(); }, [&a](bool on) { a.enable(on); }).watch();
    o.action("stop", [&a] { a.hold(); }).doc("Holds where the axis is").ui("icon", "stop");
    // Hold buttons: true while pressed, renewed within the deadman, then
    // false. Refused like target writes.
    auto jog = [&a](int direction) {
        return [&a, direction](bool pressed) {
            if (!a.owner().empty()) return Status::Busy;
            return a.jog(pressed ? direction : 0) ? Status::Ok : Status::NotAllowed;
        };
    };
    o.action("jogMinus", jog(-1)).label("Jog −").ui("widget", "hold");
    o.action("jogPlus", jog(1)).label("Jog +").ui("widget", "hold");
    o.value("position", [&a] { return a.position(); }).unit("°").step(0.01).watch().ui("widget", "chart");
    o.value(
         "target", [&a] { return a.target(); },
         [&a](float deg) -> Check {
             // The message must outlive the call; setters run under the API
             // lock, so one buffer serves every axis.
             static std::string busy;
             std::string owner = a.owner();
             if (!owner.empty()) {
                 busy = "driven by " + owner;
                 return Check::fail(Status::Busy, busy.c_str());
             }
             if (!a.setTarget(deg)) return Check::fail(Status::NotAllowed, "the axis isn't running");
             return Check::ok();
         })
        .range(0, 360)
        .unit("°")
        .step(0.01)
        .watch();
    o.value("error", [&a] { return a.error(); }).unit("°").watch();
    o.value("output", [&a] { return a.output(); }).watch().ui("widget", "chart");

    DcMotor& m = a.motor();
    auto& motor = o.object("motor");
    motor
        .value(
            "output", [&a] { return a.output(); },
            [&a](float u) -> Check {
                if (!a.setManualOutput(u)) return Check::fail(Status::NotAllowed, "the axis is enabled");
                return Check::ok();
            })
        .range(-1, 1)
        .step(0.01)
        .watch()
        .ui("keepalive", 400)  // the client renews it well within the deadman (500 ms or more)
        .doc("Drives the motor by hand while the axis is disabled, for the deadman time unless rewritten");
    motor
        .value("duty",
               [&a, &m] {
                   Guard g(a.lock());
                   return static_cast<int>(m.duty() * 100.0f + 0.5f);
               })
        .unit("%")
        .watch();
    auto& mc = motor.object("config").persist().label("Motor settings").ui("collapsed");
    setting(mc, "invert", a, m.invert).doc("Makes a positive output increase the sensor's raw angle");
    setting(mc, "minOutput", a, m.minOutput).range(0, 1).step(0.01).doc("The duty where the motor starts to move");
    setting(mc, "shape", a, m.shape).range(0.1, 5).step(0.1);

    auto& sensor = o.object("sensor");
    sensor.value("raw", [&a] { return a.raw(); }).unit("°").step(0.01).watch();
    sensor.value("faults", [&a] { return a.faults(); }).watch().doc("Failed reads since boot");
    auto& sc = sensor.object("config").persist().label("Sensor settings").ui("collapsed");
    setting(sc, "invert", a, a.sensorConfig.invert);
    setting(sc, "offset", a, a.sensorConfig.offset).range(-360, 360).unit("°").step(0.01);

    auto& limits = o.object("limits").persist().doc("In raw sensor angles: the axis stays on the arc from begin to end, increasing");
    setting(limits, "enabled", a, a.limits.enabled);
    setting(limits, "begin", a, a.limits.begin).range(0, 360).unit("°").step(0.1).ui("visibleWhen", "enabled=true");
    setting(limits, "end", a, a.limits.end).range(0, 360).unit("°").step(0.1).ui("visibleWhen", "enabled=true");

    Axis::Tuning& t = a.tuning;
    auto& tuning = o.object("tuning").persist().ui("collapsed");
    setting(tuning, "kp", a, t.kp).range(0, 10).step(0.001);
    setting(tuning, "ki", a, t.ki).range(0, 10).step(0.001);
    setting(tuning, "kd", a, t.kd).range(0, 10).step(0.001);
    setting(tuning, "tolerance", a, t.tolerance).range(0, 10).unit("°").step(0.01);
    setting(tuning, "maxSpeed", a, t.maxSpeed).range(0, 360).unit("°/s").doc("0: no slew");
    setting(tuning, "integralLimit", a, t.integralLimit).range(0, 1).step(0.01);
    setting(tuning, "derivativeFilter", a, t.derivativeFilter).range(0.01, 1).step(0.01);
    setting(tuning, "sensorTimeout", a, t.sensorTimeoutMs).range(20, 5000).unit("ms");
    setting(tuning, "jogSpeed", a, t.jogSpeed).range(0.1, 90).unit("°/s").step(0.1);
    setting(tuning, "deadman", a, t.deadmanMs)
        .range(500, 10000)
        .unit("ms")
        .doc("A manual output or a jog not renewed for this long stops");
}

}  // namespace tessermotion
