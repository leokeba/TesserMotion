// The angle correction (docs/DESIGN.md section 4.2), alone and in an axis.
#pragma once

#include <math.h>

#include <vector>

#include "sim.h"

using namespace tessermotion;

namespace {

constexpr float kDeg = 0.017453292f;

// An off-centre magnet's error: 1.5° of first harmonic, 0.4° of second.
float magnetError(float angle) {
    float t = angle * kDeg;
    return 1.5f * sinf(t + 0.5f) + 0.4f * cosf(2 * t);
}

// n points from `from` over `span` (a whole turn: evenly, without repeating
// the first), each read with `error` and referenced with a zero of `zero`.
std::vector<CalPoint> pointsOn(int n, float (*error)(float), float zero, float from = 0, float span = 360) {
    std::vector<CalPoint> points;
    float step = span / static_cast<float>(span >= 360 ? n : n - 1);
    for (int i = 0; i < n; i++) {
        float raw = wrap360(from + step * static_cast<float>(i));
        points.push_back({raw, wrap360(raw + error(raw) + zero), 1});
    }
    return points;
}

// The largest difference between the correction and `error` on an arc, up
// to a constant: on an arc short of a turn, part of the error can't be told
// from the references' zero, and goes into it.
float worst(const AngleCorrection& c, float (*error)(float), float from, float span) {
    float lo = INFINITY, hi = -INFINITY;
    for (float d = 0; d <= span; d += 0.5f) {
        float e = c.at(from + d) - error(from + d);
        lo = fminf(lo, e);
        hi = fmaxf(hi, e);
    }
    return (hi - lo) / 2;
}

}  // namespace

TEST(correction_recovers_known_harmonics) {
    std::vector<CalPoint> points = pointsOn(36, magnetError, 123);
    AngleCorrection c;
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK(fabsf(c.cos[0] - 1.5f * sinf(0.5f)) < 0.005f);
    CHECK(fabsf(c.sin[0] - 1.5f * cosf(0.5f)) < 0.005f);
    CHECK(fabsf(c.cos[1] - 0.4f) < 0.005f);
    CHECK(fabsf(c.sin[1]) < 0.005f);
    CHECK(fabsf(c.cos[2]) < 0.005f && fabsf(c.sin[2]) < 0.005f);  // three by default
    CHECK(worst(c, magnetError, 0, 360) < 0.01f);
    AngleCorrection::Residuals r = c.residuals(points.data(), points.size());
    CHECK_EQ(r.points, size_t(36));
    CHECK(fabsf(r.zero - 123) < 0.001f);  // found, not applied
    CHECK(r.rms < 0.005f && r.max < 0.01f);
    CHECK(fabsf(r.coverage - 350) < 0.001f);
    // Uncorrected, the points show the error.
    AngleCorrection none;
    CHECK(none.residuals(points.data(), points.size()).max > 1.5f);
}

TEST(correction_takes_any_zero) {
    for (float zero : {0.0f, 179.9f, 180.0f, 180.1f, 359.95f}) {
        std::vector<CalPoint> points = pointsOn(24, magnetError, zero);
        AngleCorrection c;
        CHECK(c.fit(points.data(), points.size()) == nullptr);
        CHECK(worst(c, magnetError, 0, 360) < 0.01f);
        CHECK(apart(c.residuals(points.data(), points.size()).zero, zero) < 0.001f);
    }
}

TEST(correction_inverts) {
    std::vector<CalPoint> points = pointsOn(36, magnetError, 0);
    AngleCorrection c;
    CHECK(c.fit(points.data(), points.size(), 2) == nullptr);
    float w = 0;
    for (float a = 0; a < 360; a += 0.25f) {
        w = fmaxf(w, apart(c.remove(c.apply(a)), a));
        w = fmaxf(w, apart(c.apply(c.remove(a)), a));
    }
    CHECK(w < 1e-4f);
    CHECK(c.apply(359.9f) < 360 && c.remove(0.1f) >= 0);
}

TEST(correction_stays_small_off_a_short_arc) {
    // An elevation axis: points over 90° only.
    std::vector<CalPoint> points = pointsOn(10, magnetError, 0, 0, 90);
    AngleCorrection c;
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK(worst(c, magnetError, 0, 90) < 0.02f);  // from 0.28 uncorrected
    // Undetermined away from the points, but held small by the ridge.
    float largest = 0;
    for (float a = 0; a < 360; a += 1) largest = fmaxf(largest, fabsf(c.at(a)));
    CHECK(largest < 2);
    CHECK(fabsf(c.residuals(points.data(), points.size()).coverage - 90) < 0.001f);
}

TEST(correction_weighs_points) {
    std::vector<CalPoint> points = pointsOn(12, magnetError, 0);
    AngleCorrection plain;
    CHECK(plain.fit(points.data(), points.size()) == nullptr);
    // Weight 0 leaves a wild point out.
    std::vector<CalPoint> withOut = points;
    withOut.push_back({33, 250, 0});
    AngleCorrection c;
    CHECK(c.fit(withOut.data(), withOut.size()) == nullptr);
    CHECK(fabsf(c.cos[0] - plain.cos[0]) < 1e-6f && fabsf(c.sin[1] - plain.sin[1]) < 1e-6f);
    CHECK_EQ(c.residuals(withOut.data(), withOut.size()).points, size_t(12));
    // A point twice weighs as one of weight 2.
    std::vector<CalPoint> twice = points, doubled = points;
    twice.push_back({45, 47, 1});
    twice.push_back({45, 47, 1});
    doubled.push_back({45, 47, 2});
    AngleCorrection a, b;
    CHECK(a.fit(twice.data(), twice.size()) == nullptr);
    CHECK(b.fit(doubled.data(), doubled.size()) == nullptr);
    for (size_t k = 0; k < 3; k++) CHECK(fabsf(a.cos[k] - b.cos[k]) < 1e-5f && fabsf(a.sin[k] - b.sin[k]) < 1e-5f);
    // A light point moves the fit less.
    std::vector<CalPoint> light = points;
    light.push_back({45, 47, 0.01f});
    AngleCorrection l;
    CHECK(l.fit(light.data(), light.size()) == nullptr);
    CHECK(fabsf(l.at(45) - plain.at(45)) < fabsf(a.at(45) - plain.at(45)) * 0.1f);
}

TEST(correction_chooses_harmonics_from_the_points) {
    std::vector<CalPoint> points = pointsOn(2, magnetError, 0);
    AngleCorrection c;
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK_EQ(c.harmonics(), 0);  // only the zero
    points = pointsOn(3, magnetError, 0);
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK_EQ(c.harmonics(), 1);
    points = pointsOn(13, magnetError, 0);
    CHECK(c.fit(points.data(), points.size(), 6) == nullptr);
    CHECK_EQ(c.harmonics(), 6);
}

TEST(correction_refuses_what_it_cant_fit) {
    AngleCorrection c;
    c.cos[0] = 1;
    std::vector<CalPoint> points = pointsOn(8, magnetError, 0);
    CHECK(c.fit(nullptr, 0) != nullptr);
    CHECK(c.fit(points.data(), points.size(), 4) != nullptr);   // needs 9
    CHECK(c.fit(points.data(), points.size(), 7) != nullptr);   // at most 6
    CHECK(c.fit(points.data(), points.size(), -1) != nullptr);
    std::vector<CalPoint> unused = points;
    for (CalPoint& p : unused) p.weight = 0;
    CHECK(c.fit(unused.data(), unused.size()) != nullptr);
    std::vector<CalPoint> nan = points;
    nan[3].reference = NAN;
    CHECK(c.fit(nan.data(), nan.size()) != nullptr);
    // References far off: a correction too steep to invert.
    std::vector<CalPoint> wild = points;
    wild[2].reference += 120;
    wild[5].reference -= 100;
    CHECK(c.fit(wild.data(), wild.size(), 3) != nullptr);
    // All of which left it as it was.
    CHECK_EQ(c.cos[0], 1.0f);
    CHECK_EQ(c.harmonics(), 1);
    c.clear();
    CHECK_EQ(c.harmonics(), 0);
    CHECK_EQ(c.at(42), 0.0f);
}

TEST(correction_coverage_wraps) {
    std::vector<CalPoint> points = {{350, 350, 1}, {10, 10, 1}, {30, 30, 1}};
    CHECK(fabsf(AngleCorrection().residuals(points.data(), points.size()).coverage - 40) < 0.001f);
    points.resize(1);
    CHECK_EQ(AngleCorrection().residuals(points.data(), points.size()).coverage, 0.0f);
    CHECK_EQ(AngleCorrection().residuals(nullptr, 0).points, size_t(0));
}

namespace {

PlantParams offCentre() {
    PlantParams p;
    p.sensorError = magnetError;
    return p;
}

// Drives the axis to `targets` and records a point at each, referenced to
// `truth(angle)`, the position each should show.
std::vector<CalPoint> calibrate(Sim& s, float (*truth)(float), int n) {
    std::vector<CalPoint> points;
    for (int i = 0; i < n; i++) {
        s.axis.setTarget(360.0f * static_cast<float>(i) / static_cast<float>(n));
        s.run(8);
        CalPoint p;
        CHECK(s.axis.calibrationPoint(truth(s.plant.angle()), p));
        points.push_back(p);
    }
    return points;
}

float same(float angle) { return angle; }
// An inverted sensor with an offset of 50, referenced 7° off it.
float mirrored(float angle) { return wrap360(57 - angle); }

}  // namespace

TEST(axis_correction_removes_an_off_centre_magnet) {
    Sim s(50, 10, offCentre());
    s.axis.tuning.maxSpeed = 30;
    s.start();
    CHECK(s.axis.setTarget(70));
    s.run(5);
    CHECK(apart(s.plant.angle(), 70) > 1);  // the magnet's error
    std::vector<CalPoint> points = calibrate(s, same, 24);
    CHECK(s.axis.fitCorrection(points.data(), points.size()) == nullptr);
    AngleCorrection::Residuals r = s.axis.correctionResiduals(points.data(), points.size());
    CHECK(r.rms < 0.05f && fabsf(r.zero) < 0.05f);
    for (float target : {70.0f, 133.0f, 251.0f, 359.5f}) {
        CHECK(s.axis.setTarget(target));
        s.run(10);
        CHECK(s.axis.state() == AxisState::Holding);
        CHECK(apart(s.plant.angle(), target) < 0.2f);
        CHECK(apart(s.axis.position(), target) < 0.15f);
    }
}

TEST(axis_correction_never_moves_the_motor) {
    Sim s(50, 200, offCentre());
    s.start();
    float before = s.plant.turned();
    float shown = s.axis.position();
    std::vector<CalPoint> points = pointsOn(24, [](float raw) { return -magnetError(raw); }, 0);
    CHECK(s.axis.fitCorrection(points.data(), points.size()) == nullptr);
    s.run(2);
    CHECK(fabsf(s.plant.turned() - before) < 0.05f);
    CHECK(apart(s.axis.position(), shown) > 0.5f);  // shown corrected
    CHECK(apart(s.axis.position(), 200) < 0.2f);
    CHECK(apart(s.axis.target(), s.axis.position()) < 0.1f);
}

TEST(axis_correction_follows_invert_and_offset) {
    Sim s(50, 10, offCentre());
    s.axis.sensorConfig = {true, 50};
    s.start();
    std::vector<CalPoint> points = calibrate(s, mirrored, 24);
    CHECK(s.axis.fitCorrection(points.data(), points.size()) == nullptr);
    AngleCorrection::Residuals r = s.axis.correctionResiduals(points.data(), points.size());
    CHECK(fabsf(r.zero - 7) < 0.05f);  // to add to the offset
    s.axis.sensorConfig.offset += r.zero;
    CHECK(fabsf(s.axis.correctionResiduals(points.data(), points.size()).zero) < 0.05f);
    for (float target : {20.0f, 140.0f, 300.0f}) {
        CHECK(s.axis.setTarget(target));
        s.run(15);
        CHECK(apart(s.plant.angle(), mirrored(target)) < 0.2f);
    }
}
