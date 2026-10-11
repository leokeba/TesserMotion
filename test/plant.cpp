#include "plant.h"

#include <math.h>

#include "tessermotion/angle.h"

using tessermotion::wrap360;

bool Plant::read(float& degrees) {
    reads++;
    if (failing) return false;
    float a = wrap360(angle_ + noise());
    auto counts = static_cast<uint32_t>(lroundf(a * (16384.0f / 360.0f))) & 16383u;
    degrees = static_cast<float>(counts) * (360.0f / 16384.0f);
    return true;
}

void Plant::advance(float seconds) {
    const float dt = 0.0005f;
    for (float t = 0; t < seconds - dt * 0.5f; t += dt) {
        float drive = fmaxf(0.0f, duty_ - params_.deadBand) / (1.0f - params_.deadBand);
        float target = params_.maxSpeed * drive * (forward_ ? 1.0f : -1.0f);
        speed_ += (target - speed_) * dt / params_.timeConstant;
        angle_ += speed_ * dt;
    }
}

float Plant::angle() const { return wrap360(angle_); }

float Plant::noise() {
    if (params_.noise <= 0) return 0;
    seed_ = seed_ * 1664525u + 1013904223u;  // LCG
    float u = static_cast<float>(seed_ >> 8) / static_cast<float>(1u << 24);  // 0..1
    return (2.0f * u - 1.0f) * params_.noise;
}
