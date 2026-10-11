#pragma once

// The control loop (docs/DESIGN.md sections 3.2 and 3.5): one task ticks every
// part at a fixed rate. Each tick, every part senses (bus transfers, outside
// the lock), then every part updates, under Motion's lock, in the order they
// were added.

#include <stdint.h>

#include <vector>

#include "tessermotion/platform.h"
#include "tessermotion/platform/lock.h"

namespace tessermotion {

class Motion;

class Part {
public:
    virtual ~Part() = default;

    virtual void begin() {}
    // Reads sensors; outside the lock, so a slow bus never blocks a request.
    virtual void sense(uint64_t nowUs) { (void)nowUs; }
    // Computes and applies outputs, under the lock.
    virtual void update(uint64_t nowUs) = 0;
    // Motion::stopAll(), under the lock.
    virtual void stop() {}

    // Motion's lock once added to one, else null.
    Lock* lock() const { return lock_; }

private:
    friend class Motion;
    Lock* lock_ = nullptr;
};

class Motion {
public:
    // Before begin().
    void add(Part& part);

    // Each part's begin(), in the order they were added.
    void begin();
    // begin(), then the control task: step() every 1/rateHz s (rounded to
    // the RTOS tick on ESP). False if the task couldn't start.
    bool start(float rateHz = 50, const TaskConfig& task = TaskConfig());
    // One tick; the task calls it, tests and applications without the task
    // call it themselves.
    void step(uint64_t nowUs);

    // Disables every axis; from any task.
    void stopAll();

    // Statistics, from any task.
    float periodMs();      // the last tick's period
    uint32_t tickMaxUs();  // the longest tick since the last call
    uint32_t overruns();   // ticks longer than their period, since start()

    Lock& lock() { return lock_; }

private:
    void ticked(uint64_t startUs, uint64_t endUs);

    std::vector<Part*> parts_;
    Lock lock_;
    bool begun_ = false;
    uint32_t periodUs_ = 0;  // configured by start()
    uint64_t lastUs_ = 0;
    uint32_t lastPeriodUs_ = 0;
    uint32_t tickMaxUs_ = 0;
    uint32_t overruns_ = 0;
};

}  // namespace tessermotion
