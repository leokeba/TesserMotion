#include "tesser/status.h"
#include "tessermotion/trees.h"

namespace tessermotion {

namespace {

using tesser::Check;
using tesser::Object;
using tesser::Status;

constexpr size_t kMaxAccelPoints = 32;

}  // namespace

void describe(Object& o, Tilt& t) {
    o.value("x", [&t] { return t.reading().x; }).unit("m/s²").step(0.01).watch();
    o.value("y", [&t] { return t.reading().y; }).unit("m/s²").step(0.01).watch();
    o.value("z", [&t] { return t.reading().z; }).unit("m/s²").step(0.01).watch();
    o.value("pitch", [&t] { return t.reading().pitch; }).unit("°").step(0.01).watch();
    o.value("roll", [&t] { return t.reading().roll; }).unit("°").step(0.01).watch();
    o.value("present", [&t] { return t.present(); }).watch().doc("The sensor answers");
    o.value(
         "samples",
         [&t] {
             Guard g(t.lock());
             return t.samples;
         },
         [&t](uint32_t n) {
             Guard g(t.lock());
             t.samples = n;
         })
        .range(1, Tilt::kMaxSamples)
        .persist()
        .doc("Readings averaged");

    // The calibration is the tree's, under the API lock; the bias and scale
    // are written by fit and clear under the tilt's lock, and only loading
    // writes them without it.
    Tilt::Calibration& cal = t.calibration;
    auto& corr = o.object("correction").label("Correction").ui("collapsed");
    auto& points = corr.list("points", cal.points,
                             [](Object& p, AccelPoint& point) {
                                 p.value("x", point.x).range(-160, 160).unit("m/s²").step(0.001);
                                 p.value("y", point.y).range(-160, 160).unit("m/s²").step(0.001);
                                 p.value("z", point.z).range(-160, 160).unit("m/s²").step(0.001);
                                 p.value("weight", point.weight).range(0, 100).step(0.1).doc("0 leaves the point out");
                             })
                       .maxSize(kMaxAccelPoints)
                       .persist();
    auto& bias = corr.array("bias", t.correction.bias).readOnly().persist().doc("m/s², x, y, z");
    auto& scale = corr.array("scale", t.correction.scale).readOnly().persist().doc("x, y, z");
    corr.action("record",
                [&t, &cal, &points]() -> Check {
                    AccelPoint p;
                    if (cal.points.size() >= kMaxAccelPoints) return Check::fail(Status::NotAllowed, "the list of points is full");
                    if (const char* why = t.calibrationPoint(p)) return Check::fail(Status::NotAllowed, why);
                    cal.points.push_back(p);
                    points.changed();
                    return Check::ok();
                })
        .doc("Adds a point at the current average, which must be full and still: hold the sensor in a new pose for longer than the average");
    corr.action("fit",
                [&t, &cal, &bias, &scale]() -> Check {
                    if (const char* error = t.fitCorrection(cal.points.data(), cal.points.size())) {
                        return Check::fail(Status::InvalidValue, error);
                    }
                    bias.changed();
                    scale.changed();
                    return Check::ok();
                })
        .doc("Fits the bias and scale to the points and applies them; the poses must point every axis up and down");
    corr.action("clear",
                [&t, &bias, &scale] {
                    {
                        Guard g(t.lock());
                        t.correction.clear();
                    }
                    bias.changed();
                    scale.changed();
                })
        .doc("Removes the correction; the points stay");
    // Computed on each read; watched, since the points and the correction
    // change them.
    auto& res = corr.object("residuals").doc("|corrected| − g at the points, with the correction as it is");
    res.value("points", [&t, &cal] { return static_cast<uint32_t>(t.correctionResiduals(cal.points.data(), cal.points.size()).points); })
        .watch();
    res.value("rms", [&t, &cal] { return t.correctionResiduals(cal.points.data(), cal.points.size()).rms; })
        .unit("m/s²")
        .step(0.001)
        .watch();
    res.value("max", [&t, &cal] { return t.correctionResiduals(cal.points.data(), cal.points.size()).max; })
        .unit("m/s²")
        .step(0.001)
        .watch();
}

}  // namespace tessermotion
