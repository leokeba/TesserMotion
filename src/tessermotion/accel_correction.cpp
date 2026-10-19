#include "tessermotion/accel_correction.h"

#include <math.h>

namespace tessermotion {

namespace {

// The parameters, in units of g so that all six are of order 1: the bias
// over g for x, y, z, then the scales.
constexpr int kN = 6;
using Params = double[kN];

bool usable(const AccelPoint& p) { return p.weight > 0; }

bool finite(const AccelPoint& p) { return isfinite(p.x) && isfinite(p.y) && isfinite(p.z) && isfinite(p.weight); }

// The normal equations of r = |s ⊙ (a − β)| − 1 at p: H = Σ w·JᵀJ,
// grad = Σ w·Jᵀr, and the cost Σ w·r².
struct Normal {
    double H[kN][kN] = {};
    double grad[kN] = {};
    double cost = 0;
};

Normal normal(const AccelPoint* points, size_t n, double scale, const Params p, bool withJacobian) {
    Normal e;
    const double g = AccelCorrection::kGravity;
    for (size_t i = 0; i < n; i++) {
        const AccelPoint& pt = points[i];
        if (!usable(pt)) continue;
        double w = pt.weight * scale;
        double a[3] = {pt.x / g, pt.y / g, pt.z / g};
        double d[3], c[3];
        for (int j = 0; j < 3; j++) {
            d[j] = a[j] - p[j];
            c[j] = p[3 + j] * d[j];
        }
        double m = sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
        double r = m - 1;
        e.cost += w * r * r;
        if (!withJacobian) continue;
        double J[kN];
        for (int j = 0; j < 3; j++) {
            double u = m > 0 ? c[j] / m : 0;
            J[j] = -u * p[3 + j];
            J[3 + j] = u * d[j];
        }
        for (int row = 0; row < kN; row++) {
            e.grad[row] += w * J[row] * r;
            for (int col = 0; col <= row; col++) e.H[row][col] += w * J[row] * J[col];
        }
    }
    return e;
}

// Cholesky, H = L·Lᵀ, into L's lower triangle; false unless H is positive
// definite.
bool cholesky(const double H[kN][kN], double L[kN][kN]) {
    for (int r = 0; r < kN; r++) {
        for (int c = 0; c <= r; c++) {
            double s = H[r][c];
            for (int j = 0; j < c; j++) s -= L[r][j] * L[c][j];
            if (r == c) {
                if (!(s > 0)) return false;
                L[r][r] = sqrt(s);
            } else {
                L[r][c] = s / L[c][c];
            }
        }
    }
    return true;
}

void solve(const double L[kN][kN], const double b[kN], double x[kN]) {
    for (int r = 0; r < kN; r++) {
        double s = b[r];
        for (int j = 0; j < r; j++) s -= L[r][j] * x[j];
        x[r] = s / L[r][r];
    }
    for (int r = kN - 1; r >= 0; r--) {
        double s = x[r];
        for (int j = r + 1; j < kN; j++) s -= L[j][r] * x[j];
        x[r] = s / L[r][r];
    }
}

// Whether the points determine every parameter: each one's error per unit
// of noise is √(W·(H⁻¹)ⱼⱼ), with W the sum of the weights. Names the axis
// that the points leave undetermined.
const char* determined(const Normal& e, double weights) {
    static const char* const kAxis[3] = {
        "the poses don't determine x: spread them so that x points up and down too",
        "the poses don't determine y: spread them so that y points up and down too",
        "the poses don't determine z: spread them so that z points up and down too",
    };
    double L[kN][kN] = {};
    // Before the inverse, the axis whose columns are zero: no pose moves it.
    for (int j = 0; j < 3; j++) {
        if (!(e.H[j][j] > 1e-12 * weights) || !(e.H[3 + j][3 + j] > 1e-12 * weights)) return kAxis[j];
    }
    if (!cholesky(e.H, L)) return "the poses don't determine the correction: spread them over every direction";
    double worst = 0;
    int axis = 0;
    for (int j = 0; j < kN; j++) {
        double unit[kN] = {}, x[kN];
        unit[j] = 1;
        solve(L, unit, x);
        double amplification = sqrt(weights * x[j]);
        if (amplification > worst) {
            worst = amplification;
            axis = j % 3;
        }
    }
    if (!(worst <= AccelCorrection::kMaxAmplification)) return kAxis[axis];
    return nullptr;
}

}  // namespace

const char* AccelCorrection::fit(const AccelPoint* points, size_t n) {
    size_t used = 0;
    double weights = 0;
    for (size_t i = 0; i < n; i++) {
        if (!finite(points[i])) return "a point isn't a number";
        if (!usable(points[i])) continue;
        double m = sqrt(double(points[i].x) * points[i].x + double(points[i].y) * points[i].y +
                        double(points[i].z) * points[i].z);
        if (!(m > 0.5 * kGravity && m < 1.5 * kGravity)) return "a point is far from 1 g: was the sensor still?";
        used++;
        weights += points[i].weight;
    }
    if (used < kN) return "too few points: 6 at least, spread over every direction";

    // Gauss–Newton from no correction, with the weights scaled to average
    // 1, halving a step that doesn't lower the cost.
    const double norm = static_cast<double>(used) / weights;
    const double W = static_cast<double>(used);
    Params p = {0, 0, 0, 1, 1, 1};
    Normal e = normal(points, n, norm, p, true);
    if (const char* why = determined(e, W)) return why;
    bool converged = false;
    for (int iteration = 0; iteration < 50 && !converged; iteration++) {
        double L[kN][kN] = {}, minus[kN], step[kN];
        if (!cholesky(e.H, L)) return "the poses don't determine the correction: spread them over every direction";
        for (int j = 0; j < kN; j++) minus[j] = -e.grad[j];
        solve(L, minus, step);
        double t = 1;
        Params next;
        for (int halving = 0;; halving++) {
            for (int j = 0; j < kN; j++) next[j] = p[j] + t * step[j];
            if (normal(points, n, norm, next, false).cost <= e.cost || halving == 20) break;
            t /= 2;
        }
        double moved = 0;
        for (int j = 0; j < kN; j++) {
            moved = fmax(moved, fabs(next[j] - p[j]));
            p[j] = next[j];
        }
        e = normal(points, n, norm, p, true);
        converged = moved < 1e-9;
    }
    if (!converged) return "the fit didn't converge: check the points";
    if (const char* why = determined(e, W)) return why;
    for (int j = 0; j < 3; j++) {
        if (!(fabs(p[j]) < 0.5 && p[3 + j] > 0.5 && p[3 + j] < 2)) return "the fit is implausible: check the points";
    }

    for (size_t j = 0; j < 3; j++) {
        bias[j] = static_cast<float>(p[j] * kGravity);
        scale[j] = static_cast<float>(p[3 + j]);
    }
    return nullptr;
}

AccelCorrection::Residuals AccelCorrection::residuals(const AccelPoint* points, size_t n) const {
    Residuals r;
    double weights = 0, squares = 0;
    for (size_t i = 0; i < n; i++) {
        const AccelPoint& p = points[i];
        if (!usable(p) || !finite(p)) continue;
        float x = p.x, y = p.y, z = p.z;
        apply(x, y, z);
        float d = sqrtf(x * x + y * y + z * z) - kGravity;
        squares += p.weight * d * d;
        weights += p.weight;
        r.max = fmaxf(r.max, fabsf(d));
        r.points++;
    }
    if (r.points == 0) return Residuals();
    r.rms = static_cast<float>(sqrt(squares / weights));
    return r;
}

void AccelCorrection::clear() {
    bias.fill(0);
    scale.fill(1);
}

void AccelCorrection::apply(float& x, float& y, float& z) const {
    x = scale[0] * (x - bias[0]);
    y = scale[1] * (y - bias[1]);
    z = scale[2] * (z - bias[2]);
}

}  // namespace tessermotion
