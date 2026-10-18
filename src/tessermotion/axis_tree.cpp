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

// Refuses a tree write to a claimed axis, naming the owner. The message must
// outlive the call: requests run under the API lock, so one buffer serves
// every axis.
Check claimed(Axis& a) {
    static std::string busy;
    std::string owner = a.owner();
    if (owner.empty()) return Check::ok();
    busy = "driven by " + owner;
    return Check::fail(Status::Busy, busy.c_str());
}

constexpr Check kNotRunning{Status::NotAllowed, "the axis isn't running"};

constexpr size_t kMaxCalPoints = 64;

}  // namespace

void describe(Object& o, Axis& a) {
    // Controls first, then the readouts. Only position and output are charted.
    o.value("state", [&a] { return a.state(); }, {"disabled", "starting", "holding", "moving", "sensorFault"}).watch();
    // Watched: whatever the control loop or code changes (stopAll(), the
    // manual output's timeout) reaches subscribers.
    o.value("enabled", [&a] { return a.enabled(); }, [&a](bool on) { a.enable(on); }).watch();
    o.action("stop", [&a] { a.hold(); }).doc("Stops the axis: holds where it is, ends a jog, and zeroes a hand-driven output").ui("icon", "stop");
    // Hold buttons: true while pressed, renewed within the deadman, then
    // false. Refused like target writes.
    auto jog = [&a](int direction) {
        return [&a, direction](bool pressed) {
            Check c = claimed(a);
            if (c.isOk() && !a.jog(pressed ? direction : 0)) c = kNotRunning;
            return c;
        };
    };
    o.action("jogMinus", jog(-1)).label("Jog −").ui("widget", "hold");
    o.action("jogPlus", jog(1)).label("Jog +").ui("widget", "hold");
    o.value("position", [&a] { return a.position(); }).unit("°").step(0.01).watch().ui("widget", "chart");
    o.value(
         "target", [&a] { return a.target(); },
         [&a](float deg) {
             Check c = claimed(a);
             if (c.isOk() && !a.setTarget(deg)) c = kNotRunning;
             return c;
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

    // The calibration is the tree's, under the API lock; the coefficients
    // are written by fit and clear under the axis's lock, and only loading
    // writes them without it.
    Axis::Calibration& cal = a.calibration;
    auto& corr = sensor.object("correction").label("Correction").ui("collapsed");
    corr.value("harmonics", cal.harmonics).range(0, AngleCorrection::kMaxHarmonics).persist().doc("0: as many as the points support, up to 3");
    auto& points = corr.list("points", cal.points,
                             [](Object& p, CalPoint& point) {
                                 p.value("raw", point.raw).range(0, 360).unit("°").step(0.01);
                                 p.value("reference", point.reference).range(0, 360).unit("°").step(0.01).doc("The position it should show");
                                 p.value("weight", point.weight).range(0, 100).step(0.1).doc("0 leaves the point out");
                             })
                       .maxSize(kMaxCalPoints)
                       .persist();
    auto& cosines = corr.array("cos", a.correction.cos).readOnly().persist();
    auto& sines = corr.array("sin", a.correction.sin).readOnly().persist();
    corr.action("record",
                [&a, &cal, &points](float reference) -> Check {
                    CalPoint p;
                    if (cal.points.size() >= kMaxCalPoints) return Check::fail(Status::NotAllowed, "the list of points is full");
                    if (!a.calibrationPoint(reference, p)) return Check::fail(Status::NotAllowed, "no reading yet");
                    cal.points.push_back(p);
                    points.changed();
                    return Check::ok();
                })
        .doc("Adds a point at the current reading, which should show the position given; record one several times to average");
    corr.action("fit",
                [&a, &cal, &cosines, &sines]() -> Check {
                    if (const char* error = a.fitCorrection(cal.points.data(), cal.points.size(), cal.harmonics)) {
                        return Check::fail(Status::InvalidValue, error);
                    }
                    cosines.changed();
                    sines.changed();
                    return Check::ok();
                })
        .doc("Fits the correction to the points and applies it: the position changes, the motor doesn't move");
    corr.action("clear",
                [&a, &cosines, &sines] {
                    {
                        Guard g(a.lock());
                        a.correction.clear();
                    }
                    cosines.changed();
                    sines.changed();
                })
        .doc("Removes the correction; the points stay");
    // Computed on each read; watched, since the points and the correction
    // change them.
    auto residual = [&a, &cal](float AngleCorrection::Residuals::*field) {
        return [&a, &cal, field] { return a.correctionResiduals(cal.points.data(), cal.points.size()).*field; };
    };
    auto& res = corr.object("residuals").doc("Of the points, with the correction as it is");
    res.value("points", [&a, &cal] { return static_cast<uint32_t>(a.correctionResiduals(cal.points.data(), cal.points.size()).points); })
        .watch();
    res.value("zero", residual(&AngleCorrection::Residuals::zero))
        .unit("°")
        .step(0.01)
        .watch()
        .doc("The constant left: add it to the offset if the references were absolute");
    res.value("rms", residual(&AngleCorrection::Residuals::rms)).unit("°").step(0.001).watch();
    res.value("max", residual(&AngleCorrection::Residuals::max)).unit("°").step(0.001).watch();
    res.value("coverage", residual(&AngleCorrection::Residuals::coverage)).unit("°").step(0.1).watch().doc("The arc the points span");

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
