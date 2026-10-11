#include "tessermotion/motion.h"

namespace tessermotion {

void Motion::add(Part& part) {
    part.lock_ = &lock_;
    parts_.push_back(&part);
}

void Motion::begin() {
    if (begun_) return;
    begun_ = true;
    for (Part* p : parts_) p->begin();
}

bool Motion::start(float rateHz, const TaskConfig& task) {
    if (!(rateHz > 0)) return false;
    begin();
    periodUs_ = static_cast<uint32_t>(1e6f / rateHz);
    return startPeriodic(task, periodUs_, [this](uint64_t now) {
        step(now);
        ticked(now, nowUs());
    });
}

void Motion::step(uint64_t nowUs) {
    for (Part* p : parts_) p->sense(nowUs);
    Guard g(&lock_);
    if (lastUs_) lastPeriodUs_ = static_cast<uint32_t>(nowUs - lastUs_);
    lastUs_ = nowUs;
    for (Part* p : parts_) p->update(nowUs);
}

void Motion::ticked(uint64_t startUs, uint64_t endUs) {
    uint32_t us = static_cast<uint32_t>(endUs - startUs);
    Guard g(&lock_);
    if (us > tickMaxUs_) tickMaxUs_ = us;
    if (us > periodUs_) overruns_++;
}

void Motion::stopAll() {
    Guard g(&lock_);
    for (Part* p : parts_) p->stop();
}

float Motion::periodMs() {
    Guard g(&lock_);
    return static_cast<float>(lastPeriodUs_) / 1000.0f;
}

uint32_t Motion::tickMaxUs() {
    Guard g(&lock_);
    uint32_t m = tickMaxUs_;
    tickMaxUs_ = 0;
    return m;
}

uint32_t Motion::overruns() {
    Guard g(&lock_);
    return overruns_;
}

}  // namespace tessermotion
