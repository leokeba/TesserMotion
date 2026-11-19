// Step and direction motors (docs/DESIGN.md section 4.9): the ramp, the
// motors and their shared enable line, on fake outputs and fake TMC5160s
// strapped for step and direction.
#pragma once

#include <math.h>

#include <vector>

#include <ArduinoJson.h>

#include "TesserAPI.h"

#include "fake_hw.h"
#include "fake_tmc.h"
#include "tessermotion/pulse_words.h"
#include "tessermotion/step_dir.h"
#include "tessermotion/tmc/tmc5160_map.h"
#include "tessermotion/tmc5160.h"
#include "tessermotion/trees.h"

using namespace tessermotion;

namespace {

constexpr int kEnable = 4;
constexpr int kStep1 = 12, kDir1 = 13, kStep2 = 14, kDir2 = 27;
constexpr int kCs1 = 5, kCs2 = 6;
constexpr uint32_t kStepTickUs = 20000;

// Two motors on one enable line, each with a chip strapped for step and
// direction, ticked by hand at 50 Hz.
struct StepDirRig {
    fake::Gpio gpio;
    fake::Spi bus;
    fake::Tmc5160 chip1, chip2;
    Tmc5160 driver1{bus, Tmc5160Config{kCs1, -1}, &gpio};
    Tmc5160 driver2{bus, Tmc5160Config{kCs2, -1}, &gpio};
    fake::PulseOutput out1, out2;
    StepDirGroup group{kEnable, &gpio};
    StepDirMotor motor1{out1, driver1, group, StepDirConfig{kStep1, kDir1}, &gpio};
    StepDirMotor motor2{out2, driver2, group, StepDirConfig{kStep2, kDir2}, &gpio};
    Motion motion;
    uint64_t now = 1000000;

    StepDirRig(bool pulseBeginOk = true, bool pulseStartOk = true) {
        out1.beginOk = pulseBeginOk;
        out1.startOk = pulseStartOk;
        driver1.settings.microsteps = 16;
        driver2.settings.microsteps = 16;
        chip1.stepDir = true;
        chip2.stepDir = true;
        chip1.powerOn();
        chip2.powerOn();
        chip1.attach(bus, kCs1);
        chip2.attach(bus, kCs2);
        group.add(motor1);
        group.add(motor2);
        motion.add(group);
        motion.add(driver1);
        motion.add(driver2);
        motion.add(motor1);
        motion.add(motor2);
        fake::now = now;  // the platform's clock is the ticks' clock
        motion.begin();
    }
    void tick(int n = 1) {
        for (int i = 0; i < n; i++) {
            now += kStepTickUs;
            fake::now = now;
            motion.step(now);
        }
    }
    bool dirHigh1() { return gpio.level[kDir1]; }
};

// The planner alone, at 1 MHz ticks.
constexpr uint32_t kPlannerHz = 1000000;

}  // namespace

TEST(step_planner_ramps_to_its_goal) {
    StepPlanner p;
    p.configure(1000, kPlannerHz, 2);
    p.start(1);
    uint32_t last = 0;
    for (int i = 0; i < 5000; i++) {
        uint32_t period = p.next(100);
        CHECK(period > 0);
        CHECK(p.speed() <= 100.0001f);
        if (i > 0) CHECK(period <= last + 1);  // speeding up: periods shrink
        last = period;
    }
    CHECK(fabsf(p.speed() - 100) < 1e-3f);
    CHECK_EQ(last, 10000u);  // 100 µsteps/s at 1 MHz
    // The first pulse from rest: t = √(2/a) = 44.7 ms.
    StepPlanner q;
    q.configure(1000, kPlannerHz, 2);
    q.start(1);
    CHECK(fabsf(static_cast<float>(q.next(100)) - 44721.0f) < 2);
}

TEST(step_planner_brakes_to_rest_before_it_turns) {
    StepPlanner p;
    p.configure(1000, kPlannerHz, 2);
    p.start(1);
    for (int i = 0; i < 2000; i++) p.next(100);
    CHECK(fabsf(p.speed() - 100) < 1e-3f);
    // Against the direction: the speed falls to 0, never through it.
    int pulses = 0;
    for (;;) {
        uint32_t period = p.next(-100);
        if (period == 0) break;
        CHECK(p.speed() >= 0);
        CHECK(p.dir() == 1);
        pulses++;
        CHECK(pulses < 100000);
    }
    CHECK(p.speed() == 0);
    CHECK(pulses > 0);
    // At rest, the train ends for any goal against its direction.
    CHECK_EQ(p.next(-100), 0u);
    CHECK_EQ(p.next(0), 0u);
}

TEST(step_planner_jumps_without_acceleration) {
    StepPlanner p;
    p.configure(0, kPlannerHz, 2);
    p.start(-1);
    CHECK_EQ(p.next(-500), 2000u);  // 500 µsteps/s at 1 MHz
    CHECK(fabsf(p.speed() - 500) < 1e-3f);
    CHECK_EQ(p.next(-500), 2000u);
    CHECK_EQ(p.next(500), 2000u);  // the other way: the last pulse of this direction
    CHECK(p.speed() == 0);
    CHECK_EQ(p.next(500), 0u);  // at rest: the train ends
    CHECK(p.dir() == -1);
}

TEST(step_planner_periods_at_the_longest_and_shortest) {
    StepPlanner p;
    p.configure(0, kPlannerHz, 2);
    p.start(1);
    // A speed of 0.5 µsteps/s would be 2 s: the period is at most 1 s.
    p.next(0);  // still at rest: ends
    StepPlanner slow;
    slow.configure(0, kPlannerHz, 2);
    slow.start(1);
    CHECK(slow.next(1) == 1000000u);  // 1 µstep/s
    StepPlanner fast;
    fast.configure(0, kPlannerHz, 2);
    fast.start(1);
    CHECK_EQ(fast.next(10000000), 2u);  // never shorter than minTicks
}

TEST(step_dir_plays_a_velocity_and_holds) {
    StepDirRig r;
    CHECK(r.motor1.present());
    CHECK_EQ(r.motor1.microsteps(), 16u);
    CHECK(!r.motor1.setVelocity(1000));  // disabled
    r.motor1.enable(true);
    CHECK(r.motor1.enabled());
    CHECK(r.motor2.enabled());  // one line
    CHECK(!r.gpio.level[kEnable]);
    CHECK(r.motor1.setVelocity(1000));
    CHECK(r.motor1.velocity() == 1000);
    r.tick(2);  // DIR first, then the train
    CHECK(r.out1.starts == 1);
    CHECK(r.dirHigh1());
    // Jumps: 1000 µsteps/s at 10 MHz is a period of 10000 ticks.
    r.out1.pump(10);
    for (size_t i = 0; i < 10; i++) CHECK_EQ(r.out1.periods[i], 10000u);
    CHECK(r.motor1.actualVelocity() == 1000);
    CHECK(r.out2.starts == 0);  // the other motor is still at rest
    r.motor1.hold();
    CHECK(r.motor1.velocity() == 0);
    r.tick();
    r.out1.pump(1000);
    CHECK(!r.out1.running());
    CHECK(r.motor1.actualVelocity() == 0);
    CHECK(r.motor1.enabled());  // held: enabled, at rest
}

TEST(step_dir_ramps_through_zero_and_changes_direction_at_rest) {
    StepDirRig r;
    r.motor1.ramp.acceleration = 20000;
    r.motor1.enable(true);
    CHECK(r.motor1.setVelocity(500));
    r.tick(2);
    CHECK(r.out1.running());
    r.out1.pump(200);
    CHECK(r.motor1.actualVelocity() > 0);
    CHECK(r.motor1.actualVelocity() < 500 + 1);
    // Reverse: the chip's direction waits until the train has stopped.
    CHECK(r.motor1.setVelocity(-500));
    for (int i = 0; i < 5; i++) {
        r.tick();
        if (r.out1.running()) CHECK(r.dirHigh1());
        else break;
    }
    r.out1.pump(100000);
    CHECK(!r.out1.running());
    CHECK(r.motor1.actualVelocity() == 0);
    CHECK(r.dirHigh1());  // not changed while it played
    r.tick();             // now at rest: DIR goes low
    CHECK(!r.dirHigh1());
    r.tick();             // the first pulse, after the setup time
    CHECK(r.out1.running());
    CHECK(r.out1.starts == 2);
    const size_t before = r.out1.periods.size();
    r.out1.pump(300);
    CHECK(r.out1.periods.size() == before + 300);
    CHECK(r.motor1.actualVelocity() < 0);
}

TEST(step_dir_waits_for_its_setup_time) {
    StepDirRig r;
    r.motor1.settings.dirSetupUs = 50000;  // 2.5 ticks at 50 Hz
    r.motor1.enable(true);
    r.motor1.setVelocity(100);
    r.tick();  // DIR set now
    CHECK(!r.out1.running());
    r.tick();  // 20 ms: not yet
    CHECK(!r.out1.running());
    r.tick();  // 40 ms: not yet
    CHECK(!r.out1.running());
    r.tick();  // 60 ms: started
    CHECK(r.out1.running());
}

TEST(step_dir_reports_pulse_output_failures) {
    StepDirRig beginFailure(false);
    CHECK(!beginFailure.motor1.pulseOutputReady());
    beginFailure.motor1.enable(true);
    CHECK(beginFailure.motor1.setVelocity(1000));
    beginFailure.tick(2);
    CHECK_EQ(beginFailure.out1.startAttempts, 0u);
    CHECK_EQ(beginFailure.motor1.pulseStartFailures(), 0u);

    StepDirRig startFailure(true, false);
    CHECK(startFailure.motor1.pulseOutputReady());
    startFailure.motor1.enable(true);
    CHECK(startFailure.motor1.setVelocity(1000));
    startFailure.tick(2);
    CHECK_EQ(startFailure.out1.startAttempts, 1u);
    CHECK_EQ(startFailure.motor1.pulseStartFailures(), 1u);
}

TEST(step_dir_deadman_stops_an_unrenewed_velocity) {
    StepDirRig r;
    r.motor1.ramp.acceleration = 20000;
    r.motor1.enable(true);
    r.motor1.setVelocity(300);
    r.tick(2);
    r.out1.pump(20);
    r.tick(60);  // 1.2 s without renewing
    CHECK(r.motor1.velocity() == 0);
    r.out1.pump(100000);
    CHECK(!r.out1.running());
    CHECK(r.motor1.actualVelocity() == 0);
    // Renewed, it turns on.
    r.motor1.setVelocity(300);
    r.tick(2);
    CHECK(r.out1.running());
}

TEST(step_dir_group_shares_its_enable_line) {
    StepDirRig r;
    r.motor1.enable(true);
    CHECK(r.motor2.enabled());
    CHECK(r.driver1.enabled());
    CHECK(r.driver2.enabled());  // each chip's power stage follows the group
    CHECK(r.motor2.setVelocity(-200));
    r.tick(2);
    CHECK(r.out2.running());
    r.motor2.enable(false);
    CHECK(!r.motor1.enabled());
    CHECK(r.gpio.level[kEnable]);  // high: disabled
    CHECK(!r.out2.running());
    CHECK(r.motor2.velocity() == 0);
    CHECK(!r.driver1.enabled());
    CHECK(!r.driver2.enabled());
    CHECK(!r.motor1.setVelocity(100));
}

TEST(step_dir_stop_disables_the_group) {
    StepDirRig r;
    r.motor1.enable(true);
    r.motor1.setVelocity(400);
    r.tick(2);
    CHECK(r.out1.running());
    r.motion.stopAll();
    CHECK(!r.out1.running());
    CHECK(!r.motor1.enabled());
    CHECK(!r.motor2.enabled());
    CHECK(r.gpio.level[kEnable]);
}

TEST(step_dir_clamps_velocity_and_claims) {
    StepDirRig r;
    r.motor1.enable(true);
    CHECK(r.motor1.setVelocity(1e9f));
    CHECK(r.motor1.velocity() == r.motor1.settings.maxSpeed);
    CHECK(r.motor1.setVelocity(-1e9f));
    CHECK(r.motor1.velocity() == -r.motor1.settings.maxSpeed);
    CHECK(r.motor1.owner().empty());
    r.motor1.claim("motophonique");
    CHECK(r.motor1.owner() == "motophonique");
    r.motor1.release();
    CHECK(r.motor1.owner().empty());
}

TEST(step_dir_group_refuses_a_fifth_motor) {
    StepDirGroup g(-1);
    fake::PulseOutput out;
    fake::Spi bus;
    Tmc5160 chip{bus, Tmc5160Config{7, -1}};
    StepDirMotor m(out, chip, g, StepDirConfig{1, 2});
    for (size_t i = 0; i < StepDirGroup::kMaxMotors; i++) CHECK(g.add(m));
    CHECK(!g.add(m));
}

namespace {

// A source of fixed periods, then the end.
struct ListSource : PulseSource {
    std::vector<uint32_t> periods;
    size_t next = 0;
    size_t nextPeriods(uint32_t* ticks, size_t max) override {
        if (max == 0 || next >= periods.size()) return 0;
        ticks[0] = periods[next++];
        return 1;
    }
};

}  // namespace

// Drains a source into words, `chunk` at a time.
std::vector<PulseWord> wordsOf(PulseSource& src, PulseWords& words, size_t chunk = 5) {
    std::vector<PulseWord> out;
    PulseWord buf[8];
    bool ended = false;
    size_t guard = 0;
    while (!ended && guard++ < 1000000) {
        const size_t n = words.write(src, buf, chunk < 8 ? chunk : 8, ended);
        for (size_t i = 0; i < n; i++) out.push_back(buf[i]);
    }
    return out;
}

// The words of a train: each period is a high pulse of `high` ticks, then its
// low, in words of two durations, none of them zero.
TEST(pulse_words_carry_each_period) {
    const uint32_t high = 20;
    const std::vector<uint32_t> periods = {10000, 100000, high + 2, 32767 + high, 65534 + high + 1, 1000000, 189};
    for (uint32_t p : periods) {
        ListSource one;
        one.periods = {p};
        PulseWords w(high);
        const std::vector<PulseWord> words = wordsOf(one, w);
        uint64_t total = 0;
        for (size_t i = 0; i < words.size(); i++) {
            const PulseWord& word = words[i];
            CHECK(word.duration0 >= 1 && word.duration0 <= PulseWords::kMaxDuration);
            CHECK(word.duration1 >= 1 && word.duration1 <= PulseWords::kMaxDuration);
            CHECK(!word.level1);
            CHECK_EQ(word.level0, i == 0);  // the high part opens the pulse
            total += word.duration0 + word.duration1;
        }
        // A last tick may be counted twice (a word needs two durations).
        CHECK(total == p || total == p + 1);
        CHECK(!words.empty());
    }
    // 100 000 ticks: the high, then 32 767 + 32 767 + 1 679 of low in three words.
    ListSource big;
    big.periods = {100000};
    PulseWords bw(high);
    CHECK_EQ(wordsOf(big, bw).size(), 3u);
}

TEST(pulse_words_count_each_pulse_once) {
    ListSource src;
    for (uint32_t i = 0; i < 500; i++) src.periods.push_back(40 + i * 37);
    PulseWords w(20);
    const std::vector<PulseWord> words = wordsOf(src, w, 3);
    size_t pulses = 0;
    for (const PulseWord& word : words) pulses += word.level0 ? 1 : 0;
    CHECK_EQ(pulses, 500u);
    CHECK(src.next == 500);
}

namespace {

// The rig's motors and group on a TesserAPI, as the application mounts them.
struct StepDirTreeRig : StepDirRig {
    tesser::Api api;
    StepDirTreeRig() {
        api.mount("a", motor1);
        api.mount("group", group);
    }
    struct Reply {
        tesser::Status status;
        std::string body;
    };
    Reply request(tesser::Op op, const std::string& path, const std::string& body = {}) {
        JsonDocument doc;
        if (!body.empty()) deserializeJson(doc, body);
        tesser::Request req;
        req.op = op;
        req.path = path;
        req.body = doc.as<JsonVariantConst>();
        req.client.authenticated = true;
        tesser::StringReply reply;
        api.handle(req, reply);
        return {reply.status, reply.body};
    }
    Reply get(const std::string& path) { return request(tesser::Op::Get, path); }
    Reply set(const std::string& path, const std::string& body) { return request(tesser::Op::Set, path, body); }
};

}  // namespace

TEST(step_dir_tree) {
    StepDirTreeRig r;
    CHECK_EQ(tesser::declarationErrors(), 0);
    CHECK_EQ(r.get("/a/present").body, "true");
    CHECK_EQ(r.get("/a/enabled").body, "false");
    CHECK_EQ(r.get("/a/pulseOutputReady").body, "true");
    CHECK_EQ(r.get("/a/pulseStartFailures").body, "0");
    CHECK(r.set("/a/velocity", "1000").status == tesser::Status::NotAllowed);
    CHECK(r.set("/a/enabled", "true").status == tesser::Status::Ok);
    CHECK_EQ(r.get("/group/enabled").body, "true");
    CHECK(r.set("/a/velocity", "1000").status == tesser::Status::Ok);
    CHECK_EQ(r.get("/a/velocity").body, "1000");
    CHECK_EQ(r.get("/a/microsteps").body, "16");
    // A claimed motor refuses tree writes, naming the owner.
    r.motor1.claim("motophonique");
    CHECK(r.set("/a/velocity", "500").status == tesser::Status::Busy);
    CHECK_EQ(r.get("/a/owner").body, "\"motophonique\"");
    r.motor1.release();
    CHECK(r.set("/a/velocity", "500").status == tesser::Status::Ok);
    CHECK(r.set("/a/ramp/acceleration", "20000").status == tesser::Status::Ok);
    CHECK_EQ(r.get("/a/ramp/acceleration").body, "20000");
}
