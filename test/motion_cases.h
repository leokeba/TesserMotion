// Motion: the tick, the task, statistics, stopAll.
#pragma once

#include <string>

#include "sim.h"

using namespace tessermotion;

namespace {

struct Recorder : Part {
    std::string& log;
    char name;
    Recorder(std::string& l, char n) : log(l), name(n) {}
    void begin() override { log += std::string("b") + name; }
    void sense(uint64_t) override { log += std::string("s") + name; }
    void update(uint64_t) override { log += std::string("u") + name; }
    void stop() override { log += std::string("x") + name; }
};

}  // namespace

TEST(motion_empty_ticks) {
    Motion m;
    m.begin();
    m.step(1000);
    m.step(21000);
    CHECK(fabsf(m.periodMs() - 20) < 1e-3f);
}

TEST(motion_tick_order) {
    std::string log;
    Recorder a(log, 'a'), b(log, 'b');
    Motion m;
    m.add(a);
    m.add(b);
    CHECK(a.lock() == &m.lock());
    m.begin();
    m.begin();  // once
    CHECK_EQ(log, "babb");
    log.clear();
    m.step(1000);
    CHECK_EQ(log, "sasbuaub");  // every sense, then every update
    log.clear();
    m.stopAll();
    CHECK_EQ(log, "xaxb");
}

TEST(motion_task_and_statistics) {
    fake::reset();
    std::string log;
    Recorder a(log, 'a');
    Motion m;
    m.add(a);
    CHECK(!m.start(0));
    CHECK(m.start(50));
    CHECK(fake::task.started());
    CHECK_EQ(fake::task.periodUs, 20000u);
    CHECK_EQ(log, "ba");
    fake::task.tick(300);
    fake::task.tick(25000);  // an overrun
    fake::task.tick(100);
    CHECK_EQ(m.overruns(), 1u);
    CHECK_EQ(m.tickMaxUs(), 25000u);
    CHECK_EQ(m.tickMaxUs(), 0u);  // reset on read
    CHECK(fabsf(m.periodMs() - 45) < 0.01f);  // the last tick started 25 ms late
}

TEST(motion_stop_all_disables_axes) {
    Sim s(50, 0);
    s.start();
    s.axis.setTarget(90);
    s.run(0.5f);
    s.motion.stopAll();
    s.run(0.1f);
    CHECK(!s.axis.enabled());
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
}

TEST(motion_application_part_drives_an_axis) {
    // A part added before the axis sets its target inside the tick, under
    // the lock: the lock is recursive.
    struct Follower : Part {
        Axis& axis;
        explicit Follower(Axis& a) : axis(a) {}
        void update(uint64_t nowUs) override {
            axis.setTargetNow(wrap360(static_cast<float>(nowUs) * 1e-6f * 5.0f));  // 5 °/s
        }
    };
    Plant plant(0);
    Axis axis(plant, plant);
    Follower f(axis);
    Motion m;
    m.add(f);
    m.add(axis);
    m.begin();
    axis.enable(true);
    uint64_t now = 0;
    for (int i = 0; i < 500; i++) {
        plant.advance(0.02f);
        now += 20000;
        m.step(now);
    }
    // Behind by the error P control needs for 5 °/s (about 2.8°).
    CHECK(fabsf(plant.speed() - 5) < 0.5f);
    CHECK(apart(plant.angle(), 50) < 3.5f);
}
