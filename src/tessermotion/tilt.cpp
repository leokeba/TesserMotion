#include "tessermotion/tilt.h"

#include <math.h>

namespace tessermotion {

namespace {
constexpr float kDegPerRad = 57.2957795f;
}

void Tilt::begin() {
    found_ = sensor_.begin();
    Guard g(lock());
    present_ = found_;
}

void Tilt::sense(uint64_t nowUs) {
    sampleOk_ = false;
    if (!found_) {
        if (retryUs_ == 0) {
            retryUs_ = nowUs + uint64_t(kRetryMs) * 1000;
        } else if (nowUs >= retryUs_) {
            retryUs_ = nowUs + uint64_t(kRetryMs) * 1000;
            found_ = sensor_.begin();
            failures_ = 0;
            countdown_ = 0;
        }
        return;
    }
    if (countdown_ > 0) {
        countdown_--;
        return;
    }
    countdown_ = every > 0 ? every - 1 : 0;
    sampleOk_ = sensor_.read(sample_.x, sample_.y, sample_.z);
    if (sampleOk_) {
        failures_ = 0;
    } else if (++failures_ >= kAbsentAfter) {
        found_ = false;
        retryUs_ = nowUs + uint64_t(kRetryMs) * 1000;
    }
}

void Tilt::update(uint64_t nowUs) {
    (void)nowUs;
    present_ = found_;
    if (!found_) {
        filled_ = 0;  // a sensor that comes back starts a new average
        return;
    }
    if (!sampleOk_) return;
    ring_[head_] = sample_;
    head_ = (head_ + 1) % kMaxSamples;
    if (filled_ < kMaxSamples) filled_++;
    uint32_t n = samples < 1 ? 1 : samples > kMaxSamples ? kMaxSamples : samples;
    if (n > filled_) n = filled_;
    float x = 0, y = 0, z = 0;
    for (uint32_t i = 1; i <= n; i++) {
        const Sample& s = ring_[(head_ + kMaxSamples - i) % kMaxSamples];
        x += s.x;
        y += s.y;
        z += s.z;
    }
    float k = 1.0f / static_cast<float>(n);
    reading_.x = x * k;
    reading_.y = y * k;
    reading_.z = z * k;
    reading_.pitch = atan2f(-reading_.x, sqrtf(reading_.y * reading_.y + reading_.z * reading_.z)) * kDegPerRad;
    reading_.roll = atan2f(reading_.y, reading_.z) * kDegPerRad;
}

Tilt::Reading Tilt::reading() {
    Guard g(lock());
    return reading_;
}

bool Tilt::present() {
    Guard g(lock());
    return present_;
}

}  // namespace tessermotion
