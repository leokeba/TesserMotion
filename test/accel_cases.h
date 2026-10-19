// The accelerometer's correction (docs/DESIGN.md section 6.1), alone and in
// a Tilt.
#pragma once

#include <math.h>
#include <string.h>

#include <vector>

#include "sensor_cases.h"  // its rig and fake ADXL345
#include "sim.h"

using namespace tessermotion;

namespace {

constexpr float kAccelG = AccelCorrection::kGravity;

// A sensor like the second heliostat's, which read |g| ≈ 8.5 m/s²: mostly a
// bias on z, and scales a few percent off.
const AccelCorrection& accelTruth() {
    static AccelCorrection c = [] {
        AccelCorrection t;
        t.bias = {{0.25f, -0.15f, -1.3f}};
        t.scale = {{1.03f, 0.97f, 1.06f}};
        return t;
    }();
    return c;
}

// What the sensor reads when gravity is along `u` (a unit vector), with
// `noise` m/s² of deterministic noise on each axis.
AccelPoint accelReading(const AccelCorrection& truth, const float u[3], float noise = 0) {
    static uint32_t seed = 12345;
    auto jitter = [&] {
        seed = seed * 1664525u + 1013904223u;
        return noise * (static_cast<float>(seed >> 8) / 8388608.0f - 1);
    };
    AccelPoint p;
    p.x = truth.bias[0] + kAccelG * u[0] / truth.scale[0] + jitter();
    p.y = truth.bias[1] + kAccelG * u[1] / truth.scale[1] + jitter();
    p.z = truth.bias[2] + kAccelG * u[2] / truth.scale[2] + jitter();
    return p;
}

// n poses spread evenly over the sphere (a Fibonacci lattice), those with
// `keep(u)`.
template <class Keep>
std::vector<AccelPoint> accelPoses(int n, const AccelCorrection& truth, float noise, Keep keep) {
    std::vector<AccelPoint> points;
    for (int i = 0; i < n; i++) {
        float z = 1 - 2 * (static_cast<float>(i) + 0.5f) / static_cast<float>(n);
        float r = sqrtf(1 - z * z);
        float t = 2.39996323f * static_cast<float>(i);
        float u[3] = {r * cosf(t), r * sinf(t), z};
        if (keep(u)) points.push_back(accelReading(truth, u, noise));
    }
    return points;
}

std::vector<AccelPoint> accelPoses(int n, const AccelCorrection& truth, float noise = 0) {
    return accelPoses(n, truth, noise, [](const float*) { return true; });
}

bool accelMatches(const AccelCorrection& c, const AccelCorrection& truth, float biasTol, float scaleTol) {
    for (size_t j = 0; j < 3; j++) {
        if (!(fabsf(c.bias[j] - truth.bias[j]) < biasTol && fabsf(c.scale[j] - truth.scale[j]) < scaleTol)) return false;
    }
    return true;
}

bool accelSays(const char* why, const char* what) { return why && strstr(why, what); }

}  // namespace

TEST(accel_fit_recovers_bias_and_scale) {
    std::vector<AccelPoint> points = accelPoses(14, accelTruth(), 0.02f);
    AccelCorrection c;
    AccelCorrection::Residuals before = c.residuals(points.data(), points.size());
    CHECK_EQ(before.points, size_t(14));
    CHECK(before.rms > 0.5f);
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK(accelMatches(c, accelTruth(), 0.02f, 0.003f));
    AccelCorrection::Residuals after = c.residuals(points.data(), points.size());
    CHECK(after.rms < 0.02f);
    CHECK(after.max < 0.04f);
    // Applied: any pose reads g, along it.
    float u[3] = {0.48f, -0.6f, 0.64f};
    AccelPoint p = accelReading(accelTruth(), u);
    c.apply(p.x, p.y, p.z);
    CHECK(fabsf(p.x - kAccelG * u[0]) < 0.02f && fabsf(p.y - kAccelG * u[1]) < 0.02f && fabsf(p.z - kAccelG * u[2]) < 0.02f);
}

TEST(accel_fit_six_poses) {
    // A tumble is six well-spread points, and exact ones are fitted exactly.
    std::vector<AccelPoint> points;
    for (int axis = 0; axis < 3; axis++) {
        for (float sign : {1.0f, -1.0f}) {
            float u[3] = {0, 0, 0};
            u[axis] = sign;
            points.push_back(accelReading(accelTruth(), u));
        }
    }
    AccelCorrection c;
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK(accelMatches(c, accelTruth(), 1e-4f, 1e-5f));
    CHECK(c.residuals(points.data(), points.size()).max < 1e-4f);
}

TEST(accel_fit_refuses_what_it_cant_fit) {
    AccelCorrection c;
    c.bias = {{0.1f, 0.2f, 0.3f}};
    const AccelCorrection kept = c;
    auto unchanged = [&] { return c.bias == kept.bias && c.scale == kept.scale; };
    std::vector<AccelPoint> points = accelPoses(12, accelTruth());
    CHECK(accelSays(c.fit(points.data(), 5), "too few points"));
    // With five left out, too.
    std::vector<AccelPoint> light = points;
    for (size_t i = 0; i < 7; i++) light[i].weight = 0;
    CHECK(accelSays(c.fit(light.data(), light.size()), "too few points"));
    std::vector<AccelPoint> bad = points;
    bad[3].y = NAN;
    CHECK(accelSays(c.fit(bad.data(), bad.size()), "isn't a number"));
    bad = points;
    bad[3] = {0.5f, 0.3f, 2.0f, 1};  // falling, or knocked
    CHECK(accelSays(c.fit(bad.data(), bad.size()), "far from 1 g"));
    // Turned about y only, so y never moves: as on an elevation axis.
    std::vector<AccelPoint> plane;
    for (int i = 0; i < 12; i++) {
        float t = 0.5236f * static_cast<float>(i);
        float u[3] = {cosf(t), 0, sinf(t)};
        plane.push_back(accelReading(accelTruth(), u));
    }
    CHECK(accelSays(c.fit(plane.data(), plane.size()), "don't determine y"));
    // z always up: its bias can't be told from its scale.
    std::vector<AccelPoint> upper = accelPoses(40, accelTruth(), 0, [](const float* u) { return u[2] > 0; });
    CHECK(accelSays(c.fit(upper.data(), upper.size()), "don't determine z"));
    // A little below the horizon is enough.
    std::vector<AccelPoint> most = accelPoses(40, accelTruth(), 0, [](const float* u) { return u[2] > -0.5f; });
    CHECK(unchanged());
    CHECK(c.fit(most.data(), most.size()) == nullptr);
    CHECK(accelMatches(c, accelTruth(), 1e-3f, 1e-4f));
}

TEST(accel_fit_weighs_points) {
    std::vector<AccelPoint> points = accelPoses(12, accelTruth());
    AccelCorrection clean;
    CHECK(clean.fit(points.data(), points.size()) == nullptr);
    // A point taken while the sensor moved, left out.
    points.push_back({3, -2, 8, 0});
    AccelCorrection c;
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK(accelMatches(c, clean, 1e-5f, 1e-6f));
    CHECK_EQ(c.residuals(points.data(), points.size()).points, size_t(12));
    // Counted, it pulls the fit off.
    points.back().weight = 1;
    CHECK(c.fit(points.data(), points.size()) == nullptr);
    CHECK(!accelMatches(c, clean, 0.05f, 0.005f));
    CHECK(c.residuals(points.data(), points.size()).max > 0.2f);
}

TEST(tilt_applies_the_correction) {
    fake::I2c bus;
    // Level, but with a bias on x and on z: a pitch of −1.56° uncorrected.
    adxl345Device(bus, 6, 0, 220);
    Adxl345 accel(bus);
    Tilt tilt(accel);
    SensorRig r(tilt);
    r.run(1);
    Tilt::Reading t = tilt.reading();
    CHECK(fabsf(t.pitch + 1.56f) < 0.01f);
    {
        Guard g(tilt.lock());
        tilt.correction.bias = {{6 * 0.0039f * 9.80665f, 0, -0.5f}};
        tilt.correction.scale = {{1, 1, 1.1f}};
    }
    r.run(0.1f);  // applied from the next average
    t = tilt.reading();
    CHECK(fabsf(t.x) < 1e-4f);
    CHECK(fabsf(t.z - 1.1f * (220 * 0.0039f * 9.80665f + 0.5f)) < 1e-3f);
    CHECK(fabsf(t.pitch) < 1e-3f);
}

TEST(tilt_records_still_points) {
    fake::I2c bus;
    Adxl345 accel(bus);
    Tilt tilt(accel);
    SensorRig r(tilt);
    AccelPoint p;
    CHECK(accelSays(tilt.calibrationPoint(p), "no sensor"));
    adxl345Device(bus, 0, 0, 256);
    r.run(2.1f);  // found at the next probe
    CHECK(tilt.present());
    CHECK(accelSays(tilt.calibrationPoint(p), "isn't full"));
    r.run(1);  // 8 readings at 10 Hz
    {
        Guard g(tilt.lock());
        tilt.correction.bias = {{1, 1, 1}};
    }
    CHECK(tilt.calibrationPoint(p) == nullptr);
    CHECK(fabsf(p.z - kG) < 1e-4f);  // uncorrected
    CHECK(fabsf(p.x) < 1e-4f && p.weight == 1);
    // Turned: refused until the average holds only the new pose.
    adxl345Device(bus, 0, 256, 0);
    r.run(0.3f);
    CHECK(accelSays(tilt.calibrationPoint(p), "moved"));
    r.run(1);
    CHECK(tilt.calibrationPoint(p) == nullptr);
    CHECK(fabsf(p.y - kG) < 1e-4f && fabsf(p.z) < 1e-4f);
}

TEST(tilt_fits_its_correction) {
    fake::I2c bus;
    Adxl345 accel(bus);
    Tilt tilt(accel);
    std::vector<AccelPoint> points = accelPoses(14, accelTruth(), 0.02f);
    CHECK(tilt.correctionResiduals(points.data(), points.size()).rms > 0.5f);
    CHECK(accelSays(tilt.fitCorrection(points.data(), 3), "too few"));
    CHECK(tilt.correction.scale[0] == 1);
    CHECK(tilt.fitCorrection(points.data(), points.size()) == nullptr);
    CHECK(accelMatches(tilt.correction, accelTruth(), 0.02f, 0.003f));
    CHECK(tilt.correctionResiduals(points.data(), points.size()).rms < 0.02f);
}
