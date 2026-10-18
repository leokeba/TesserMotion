#include "tessermotion/angle_correction.h"

#include <math.h>

#include <algorithm>
#include <vector>

#include "tessermotion/angle.h"

namespace tessermotion {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadD = kPi / 180;
constexpr float kRad = static_cast<float>(kRadD);
// The harmonics' penalty, per point: negligible where the points determine
// them (a full turn of points gives each about n/2), decisive where they
// don't (points on a short arc), so the fit stays small away from the points.
constexpr double kRidge = 1e-3;
// Bound on |c′|, under which remove() converges (by a factor |c′| per step).
// A real sensor is far below: 2° of first harmonic is 0.035.
constexpr double kMaxSlope = 0.5;

bool usable(const CalPoint& p) { return p.weight > 0; }

bool finite(const CalPoint& p) { return isfinite(p.raw) && isfinite(p.reference) && isfinite(p.weight); }

// The weighted mean of angles around a circle: the circular mean, then the
// arithmetic one around it, so close angles near ±180 average as they
// should. `d(p)` gives each point's angle.
template <class D>
float meanAngle(const CalPoint* points, size_t n, D d) {
    double s = 0, c = 0, w = 0;
    for (size_t i = 0; i < n; i++) {
        if (!usable(points[i])) continue;
        double a = static_cast<double>(d(points[i])) * kRadD;
        s += points[i].weight * ::sin(a);
        c += points[i].weight * ::cos(a);
    }
    float guess = static_cast<float>(atan2(s, c) / kRadD);
    double sum = 0;
    for (size_t i = 0; i < n; i++) {
        if (!usable(points[i])) continue;
        sum += points[i].weight * wrap180(d(points[i]) - guess);
        w += points[i].weight;
    }
    return w > 0 ? guess + static_cast<float>(sum / w) : 0;
}

}  // namespace

const char* AngleCorrection::fit(const CalPoint* points, size_t n, int harmonics) {
    if (harmonics < 0 || harmonics > kMaxHarmonics) return "harmonics must be 0 to 6";
    size_t used = 0;
    double weights = 0;
    for (size_t i = 0; i < n; i++) {
        if (!finite(points[i])) return "a point isn't a number";
        if (!usable(points[i])) continue;
        used++;
        weights += points[i].weight;
    }
    if (used == 0) return "no points";
    int k = harmonics ? harmonics : std::min(kAutoHarmonics, static_cast<int>((used - 1) / 2));
    if (used < static_cast<size_t>(2 * k + 1)) return "too few points: n harmonics need 2n + 1";

    // Normal equations of e = a₀ + Σ a_k cos kθ + b_k sin kθ, with e the
    // reference − raw unwrapped around their mean, and weights scaled to
    // average 1 so that the ridge's scale doesn't depend on them.
    constexpr int kMax = 2 * kMaxHarmonics + 1;
    const int m = 2 * k + 1;
    double A[kMax][kMax] = {};
    double y[kMax] = {};
    const float zero = meanAngle(points, n, [](const CalPoint& p) { return p.reference - p.raw; });
    const double scale = static_cast<double>(used) / weights;
    for (size_t i = 0; i < n; i++) {
        const CalPoint& p = points[i];
        if (!usable(p)) continue;
        double w = p.weight * scale;
        double e = zero + wrap180(p.reference - p.raw - zero);
        double phi[kMax];
        phi[0] = 1;
        double t = static_cast<double>(p.raw) * kRadD;
        for (int j = 1; j <= k; j++) {
            phi[2 * j - 1] = ::cos(j * t);
            phi[2 * j] = ::sin(j * t);
        }
        for (int r = 0; r < m; r++) {
            y[r] += w * phi[r] * e;
            for (int c = 0; c <= r; c++) A[r][c] += w * phi[r] * phi[c];
        }
    }
    for (int r = 1; r < m; r++) A[r][r] += kRidge * static_cast<double>(used);

    // Cholesky, A = L·Lᵀ in the lower triangle, then the two substitutions.
    for (int r = 0; r < m; r++) {
        for (int c = 0; c <= r; c++) {
            double s = A[r][c];
            for (int j = 0; j < c; j++) s -= A[r][j] * A[c][j];
            if (r == c) {
                if (!(s > 1e-12 * static_cast<double>(used))) return "the points don't determine the correction";
                A[r][r] = sqrt(s);
            } else {
                A[r][c] = s / A[c][c];
            }
        }
    }
    double x[kMax];
    for (int r = 0; r < m; r++) {
        double s = y[r];
        for (int j = 0; j < r; j++) s -= A[r][j] * x[j];
        x[r] = s / A[r][r];
    }
    for (int r = m - 1; r >= 0; r--) {
        double s = x[r];
        for (int j = r + 1; j < m; j++) s -= A[j][r] * x[j];
        x[r] = s / A[r][r];
    }

    double slope = 0;
    for (int j = 1; j <= k; j++) slope += j * hypot(x[2 * j - 1], x[2 * j]) * kRadD;
    if (!(slope < kMaxSlope)) return "the correction would be too steep: check the references";

    clear();
    for (int j = 1; j <= k; j++) {
        cos[static_cast<size_t>(j - 1)] = static_cast<float>(x[2 * j - 1]);
        sin[static_cast<size_t>(j - 1)] = static_cast<float>(x[2 * j]);
    }
    return nullptr;
}

AngleCorrection::Residuals AngleCorrection::residuals(const CalPoint* points, size_t n) const {
    Residuals r;
    std::vector<float> angles;
    double weights = 0, squares = 0;
    auto left = [this](const CalPoint& p) { return p.reference - p.raw - at(p.raw); };
    r.zero = meanAngle(points, n, left);
    for (size_t i = 0; i < n; i++) {
        const CalPoint& p = points[i];
        if (!usable(p) || !finite(p)) continue;
        float d = wrap180(left(p) - r.zero);
        squares += p.weight * d * d;
        weights += p.weight;
        r.max = fmaxf(r.max, fabsf(d));
        angles.push_back(wrap360(p.raw));
    }
    r.points = angles.size();
    if (r.points == 0) return Residuals();
    r.rms = static_cast<float>(sqrt(squares / weights));
    // The arc is the circle less the widest gap between neighbours.
    std::sort(angles.begin(), angles.end());
    float gap = angles.front() + 360.0f - angles.back();
    for (size_t i = 1; i < angles.size(); i++) gap = fmaxf(gap, angles[i] - angles[i - 1]);
    r.coverage = 360.0f - gap;
    return r;
}

void AngleCorrection::clear() {
    cos.fill(0);
    sin.fill(0);
}

int AngleCorrection::harmonics() const {
    for (int k = kMaxHarmonics; k > 0; k--) {
        if (cos[static_cast<size_t>(k - 1)] != 0 || sin[static_cast<size_t>(k - 1)] != 0) return k;
    }
    return 0;
}

float AngleCorrection::at(float raw) const {
    int k = harmonics();
    if (k == 0) return 0;
    // cos kθ and sin kθ by the angle sum, from cos θ and sin θ.
    float t = raw * kRad;
    float c1 = cosf(t), s1 = sinf(t);
    float c = c1, s = s1, sum = 0;
    for (size_t j = 0; j < static_cast<size_t>(k); j++) {
        sum += cos[j] * c + sin[j] * s;
        float next = c * c1 - s * s1;
        s = s * c1 + c * s1;
        c = next;
    }
    return sum;
}

float AngleCorrection::apply(float raw) const { return wrap360(raw + at(raw)); }

float AngleCorrection::remove(float corrected) const {
    // raw = corrected − c(raw), by fixed-point steps: each shrinks the error
    // by |c′|, so a few give float precision.
    float raw = corrected - at(corrected);
    for (int i = 0; i < 8; i++) {
        float next = corrected - at(raw);
        bool done = fabsf(next - raw) < 1e-5f;
        raw = next;
        if (done) break;
    }
    return wrap360(raw);
}

}  // namespace tessermotion
