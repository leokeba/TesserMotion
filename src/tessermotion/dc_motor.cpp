#include "tessermotion/dc_motor.h"

#include <math.h>

namespace tessermotion {

void DcMotor::output(float u) {
    if (!(u == u)) u = 0;  // NaN
    u = fminf(fmaxf(u, -1.0f), 1.0f);
    float a = fabsf(u);
    output_ = u;
    if (a < 0.001f) {
        // Keep the direction: only the duty drops.
        duty_ = 0;
    } else {
        duty_ = powf(a, shape) * (1.0f - minOutput) + minOutput;
        forward_ = u > 0;
    }
    drive(forward_ != invert, duty_);
}

}  // namespace tessermotion
