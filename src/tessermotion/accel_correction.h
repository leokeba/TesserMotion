#pragma once

// A correction of an accelerometer's bias and scale (docs/DESIGN.md section
// 6.1), fitted to still readings in poses spread over every direction: at
// rest, a perfect sensor reads g in any pose, so the readings lie on an
// ellipsoid that the correction turns into a sphere of radius g.
//
//   corrected_i = scale_i · (raw_i − bias_i),  i = x, y, z

#include <stddef.h>

#include <array>

namespace tessermotion {

// A still reading, m/s². Weight 0 leaves a point out; a pose recorded
// several times counts as their average.
struct AccelPoint {
    float x = 0, y = 0, z = 0;
    float weight = 1;
};

class AccelCorrection {
public:
    static constexpr float kGravity = 9.80665f;  // m/s², the sphere's radius
    // Each parameter's error, per unit of noise in the readings, that a fit
    // accepts: 1.7 for the six poses of a tumble, about 2.5 for poses spread
    // evenly, 16 for poses on one hemisphere only.
    static constexpr float kMaxAmplification = 8;

    // How well a correction explains a set of points.
    struct Residuals {
        size_t points = 0;  // with a weight above 0
        float rms = 0;      // m/s²: of |corrected| − g
        float max = 0;      // m/s²: the largest of those, unsigned
    };

    std::array<float, 3> bias{};            // m/s²
    std::array<float, 3> scale{{1, 1, 1}};

    // Fits the bias and scale to the points. On success replaces them and
    // returns nullptr; otherwise keeps them and returns why.
    const char* fit(const AccelPoint* points, size_t n);
    Residuals residuals(const AccelPoint* points, size_t n) const;
    void clear();

    void apply(float& x, float& y, float& z) const;
};

}  // namespace tessermotion
