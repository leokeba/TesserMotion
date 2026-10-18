#pragma once

// A correction of an angle sensor's error, such as an off-centre magnet's
// (docs/DESIGN.md section 4.2): a short Fourier series in the raw angle,
// fitted to reference points by weighted least squares.
//
//   corrected = raw + c(raw),  c(θ) = Σ_k a_k·cos kθ + b_k·sin kθ,  k = 1..K
//
// The fit also finds the constant a₀ between the references and the
// readings, but doesn't apply it: the sensor's offset stays the zero, and
// references with any zero (steps counted from wherever a stepper started)
// do.

#include <stddef.h>

#include <array>

namespace tessermotion {

// A reading and what a perfect sensor would have read there, both in the
// raw frame (the reference with any zero). Weight 0 leaves a point out; a
// point recorded several times counts as their average.
struct CalPoint {
    float raw = 0;
    float reference = 0;
    float weight = 1;
};

class AngleCorrection {
public:
    static constexpr int kMaxHarmonics = 6;
    // What an automatic fit uses at most.
    static constexpr int kAutoHarmonics = 3;

    // How well a correction explains a set of points.
    struct Residuals {
        size_t points = 0;   // with a weight above 0
        float zero = 0;      // °: the constant left, reference − corrected (the fit's a₀)
        float rms = 0;       // °: of what's left once `zero` is taken out
        float max = 0;       // °: the largest of those, unsigned
        float coverage = 0;  // °: the smallest arc holding every point's raw angle
    };

    // Coefficients in degrees: cos[k−1] = a_k, sin[k−1] = b_k.
    std::array<float, kMaxHarmonics> cos{};
    std::array<float, kMaxHarmonics> sin{};

    // Fits `harmonics` (0: as many as the points support, up to
    // kAutoHarmonics) to the points. On success replaces the coefficients and
    // returns nullptr; otherwise keeps them and returns why.
    const char* fit(const CalPoint* points, size_t n, int harmonics = 0);
    Residuals residuals(const CalPoint* points, size_t n) const;
    void clear();
    // The highest harmonic in use; 0 for none.
    int harmonics() const;

    float at(float raw) const;  // c(raw), °
    // raw + c(raw), 0..360.
    float apply(float raw) const;
    // The raw angle that corrects to `corrected`, 0..360.
    float remove(float corrected) const;
};

}  // namespace tessermotion
