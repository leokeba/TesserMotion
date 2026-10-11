#pragma once

// Angles on a circle, in degrees.

#include <math.h>

namespace tessermotion {

// 0 <= result < 360.
inline float wrap360(float a) {
    float r = fmodf(a, 360.0f);
    if (r < 0) r += 360.0f;
    return r >= 360.0f ? 0.0f : r;  // a tiny negative r rounds up to 360
}

// -180 <= result < 180.
inline float wrap180(float a) { return wrap360(a + 180.0f) - 180.0f; }

// An arc from `begin` to `end` in increasing angles. begin == end, or
// disabled: inactive (the whole circle).
struct Arc {
    bool active;
    float middle;
    float half;  // half its span

    Arc(bool enabled, float begin, float end) {
        float span = wrap360(end - begin);
        active = enabled && span > 0;
        half = span * 0.5f;
        middle = wrap360(begin + half);
    }
};

}  // namespace tessermotion
