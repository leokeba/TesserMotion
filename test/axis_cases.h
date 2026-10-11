// The axis on the simulated plant (docs/DESIGN.md section 11).
#pragma once

#include <stdlib.h>

#include "sim.h"

using namespace tessermotion;

namespace {

void reachesAndHolds(float rateHz) {
    Sim s(rateHz, 10);
    s.start();
    for (float target : {90.0f, 270.0f, 359.9f, 0.05f, 180.0f}) {
        CHECK(s.axis.setTarget(target));
        s.run(10);
        CHECK(s.axis.state() == AxisState::Holding);
        CHECK(apart(s.plant.angle(), target) < 0.15f);
        // And stays.
        bool held = true;
        s.run(3, [&] { held = held && apart(s.plant.angle(), target) < 0.2f; });
        CHECK(held);
    }
}

}  // namespace

TEST(axis_reaches_and_holds_at_50hz) { reachesAndHolds(50); }
TEST(axis_reaches_and_holds_at_20hz) { reachesAndHolds(20); }

TEST(axis_takes_the_short_way_without_limits) {
    Sim s(50, 350);
    s.start();
    float lowest = 0;
    s.axis.setTarget(10);
    s.run(5, [&] { lowest = fminf(lowest, s.plant.turned() - 350); });
    CHECK(apart(s.plant.angle(), 10) < 0.15f);
    CHECK(s.plant.turned() > 359);  // through 0, forwards
    CHECK(lowest > -0.5f);
}

TEST(axis_starts_without_jumping) {
    Sim s(50, 123);
    s.plant.failing = true;
    s.axis.enable(true);
    s.run(1);
    CHECK(s.axis.state() == AxisState::Starting);
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
    CHECK(!s.axis.setTarget(10));  // not running yet
    s.plant.failing = false;
    s.run(2);
    CHECK(s.axis.state() == AxisState::Holding);
    CHECK(apart(s.axis.target(), 123) < 0.05f);
    CHECK(fabsf(s.plant.turned() - 123) < 0.1f);
}

TEST(axis_never_crosses_its_limits) {
    srand(7);
    for (int round = 0; round < 4; round++) {
        Sim s(50, 100, PlantParams(), static_cast<uint32_t>(round + 1));
        // The forbidden side is 300..30 through 0, in the raw frame.
        s.axis.limits = {true, 30, 300};
        s.axis.sensorConfig.offset = static_cast<float>(rand() % 720 - 360);
        s.axis.sensorConfig.invert = round % 2 == 1;
        s.axis.tuning.maxSpeed = round < 2 ? 0 : 60;
        s.start();
        bool inside = true;
        for (int i = 0; i < 10; i++) {
            float target = static_cast<float>(rand() % 3600) / 10;
            CHECK(s.axis.setTarget(target));
            s.run(15, [&] { inside = inside && fabsf(wrap180(s.plant.angle() - 165)) < 135.5f; });
            // Reached, or stopped at the nearer end of the arc.
            float raw = s.plant.angle();
            CHECK(apart(raw, s.axis.raw()) < 0.1f);
            CHECK(apart(s.axis.position(), s.axis.target()) < 0.15f);
        }
        CHECK(inside);
    }
}

TEST(axis_limits_clamp_to_the_nearer_end) {
    Sim s(50, 100);
    s.axis.limits = {true, 30, 300};
    s.start();
    s.axis.setTarget(310);
    CHECK(fabsf(s.axis.target() - 300) < 1e-3f);
    s.axis.setTarget(20);
    CHECK(fabsf(s.axis.target() - 30) < 1e-3f);
    s.run(10);
    CHECK(apart(s.plant.angle(), 30) < 0.15f);
    // begin == end: no limits.
    s.axis.limits = {true, 50, 50};
    s.axis.setTarget(0);
    CHECK(fabsf(s.axis.target()) < 1e-3f);
}

TEST(axis_offset_never_moves_the_motor) {
    Sim s(50, 100);
    s.axis.limits = {true, 30, 300};
    s.start();
    s.axis.setTarget(200);
    s.run(8);
    float before = s.plant.turned();
    s.axis.sensorConfig.offset = 50;
    s.run(2);
    CHECK(fabsf(s.plant.turned() - before) < 0.2f);
    CHECK(apart(s.axis.position(), 250) < 0.15f);
    CHECK(apart(s.axis.target(), 250) < 0.01f);
    s.axis.sensorConfig.invert = true;  // position = −raw + offset
    s.run(2);
    CHECK(fabsf(s.plant.turned() - before) < 0.2f);
    CHECK(apart(s.axis.position(), -200 + 50) < 0.15f);
    // The limits stay at the same stops: raw 30..300, shown 20..290 the
    // other way round. Raw 320 is forbidden, nearer to 300.
    s.axis.setTarget(wrap360(-320 + 50));
    CHECK(apart(s.axis.target(), wrap360(-300 + 50)) < 0.01f);
}

TEST(axis_slews_at_max_speed) {
    Sim s(50, 0);
    s.axis.tuning.maxSpeed = 10;
    s.start();
    s.axis.setTarget(90);
    s.run(2);
    // The plant follows the setpoint's ramp, behind it by the error P
    // control needs for that speed.
    CHECK(s.axis.state() == AxisState::Moving);
    CHECK(fabsf(s.plant.speed() - 10) < 0.5f);
    CHECK(s.plant.turned() < 20);
    // A new speed applies from the setpoint where it is.
    s.axis.tuning.maxSpeed = 20;
    s.run(2);
    CHECK(fabsf(s.plant.speed() - 20) < 1);
    s.run(4);
    CHECK(s.axis.state() == AxisState::Holding);
    CHECK(apart(s.plant.angle(), 90) < 0.15f);
    // setTargetNow skips the slew.
    s.axis.setTargetNow(130);
    s.run(0.5f);
    CHECK(s.plant.turned() > 100);
}

TEST(axis_stops_on_sensor_timeout_and_resumes) {
    Sim s(50, 0);
    s.start();
    s.axis.setTarget(90);
    s.run(1);
    s.plant.failing = true;
    s.run(0.1f);
    CHECK(s.axis.state() == AxisState::Moving);  // within the timeout
    s.run(0.2f);
    CHECK(s.axis.state() == AxisState::SensorFault);
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
    CHECK(s.axis.faults() >= 15u);
    s.plant.failing = false;
    s.run(8);
    CHECK(s.axis.state() == AxisState::Holding);
    CHECK(apart(s.plant.angle(), 90) < 0.15f);
}

TEST(axis_disabled_outputs_zero) {
    Sim s(50, 0);
    s.start();
    s.axis.setTarget(180);
    s.run(1);
    CHECK(s.plant.drivenDuty() > 0);
    CHECK(!s.axis.setManualOutput(0.5f));  // enabled
    s.axis.enable(false);
    s.run(0.1f);
    CHECK(s.axis.state() == AxisState::Disabled);
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
    CHECK(!s.axis.setTarget(10));
    // By hand while disabled; the target follows the position.
    CHECK(s.axis.setManualOutput(0.5f));
    s.run(0.5f);
    CHECK(s.plant.drivenDuty() > 0.5f);
    CHECK(apart(s.axis.target(), s.axis.position()) < 0.01f);
    // Enabling holds where it is, the manual output dropped.
    s.axis.enable(true);
    s.run(1);
    float here = s.plant.turned();
    s.run(1);
    CHECK(fabsf(s.plant.turned() - here) < 0.2f);
    CHECK(s.axis.state() == AxisState::Holding);
    s.axis.enable(false);
    s.run(0.1f);
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
}

TEST(axis_hold_stops_a_slew) {
    Sim s(50, 0);
    s.axis.tuning.maxSpeed = 10;
    s.start();
    s.axis.setTarget(90);
    s.run(1);
    s.axis.hold();
    s.run(2);
    CHECK(s.plant.turned() < 13);
    CHECK(s.axis.state() == AxisState::Holding);
}

TEST(axis_pid_terms) {
    // The integral removes an offset the proportional term leaves: a plant
    // with a dead band above minOutput stalls under P alone.
    PlantParams p;
    p.deadBand = 0.3f;
    Sim s(50, 0, p);
    s.axis.tuning.tolerance = 0.05f;
    s.start();
    s.axis.setTarget(30);
    s.run(10);
    CHECK(s.axis.state() == AxisState::Moving);  // stalled outside tolerance
    s.axis.tuning.ki = 0.05f;
    s.run(10);
    CHECK(apart(s.plant.angle(), 30) < 0.1f);
}

TEST(axis_manual_output_times_out) {
    Sim s(50, 0);
    s.run(0.1f);
    CHECK(s.axis.setManualOutput(0.4f));
    s.run(0.9f);
    CHECK(s.plant.drivenDuty() > 0.4f);
    s.run(0.2f);  // not rewritten for 1 s: the deadman
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
    CHECK_EQ(s.axis.output(), 0.0f);
    // Rewriting keeps it going.
    for (int i = 0; i < 6; i++) {
        CHECK(s.axis.setManualOutput(-0.4f));
        s.run(0.5f);
        CHECK(s.plant.drivenDuty() > 0.4f);
    }
    s.axis.tuning.deadmanMs = 200;
    s.axis.setManualOutput(0.4f);
    s.run(0.3f);
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
}

TEST(axis_jogs_at_jog_speed) {
    Sim s(50, 100);
    s.axis.tuning.jogSpeed = 5;
    CHECK(!s.axis.jog(1));  // not running
    s.start();
    // Renewed every 300 ms as a hold button does: 3 s at 5 °/s.
    for (int i = 0; i < 10; i++) {
        CHECK(s.axis.jog(1));
        s.run(0.3f);
        CHECK(s.axis.state() == AxisState::Moving);
    }
    CHECK(s.axis.jog(0));
    CHECK(fabsf(s.axis.target() - 115) < 0.2f);
    s.run(3);
    CHECK(s.axis.state() == AxisState::Holding);
    CHECK(apart(s.plant.angle(), 115) < 0.15f);
    // Backwards, in the output frame even when inverted.
    s.axis.sensorConfig.invert = true;
    s.axis.sensorConfig.offset = 360;
    float shown = s.axis.position();
    s.axis.jog(-1);
    s.run(0.5f);
    s.axis.jog(0);
    s.run(2);
    CHECK(fabsf(wrap180(s.axis.position() - (shown - 2.5f))) < 0.3f);
}

TEST(axis_jog_deadman) {
    Sim s(50, 100);
    s.start();
    s.axis.jog(1);  // never released
    s.run(3);
    CHECK(s.axis.state() == AxisState::Holding);
    CHECK(fabsf(s.plant.turned() - 105) < 0.3f);  // 1 s at 5 °/s
}

TEST(axis_jog_stops_at_limits_and_on_commands) {
    Sim s(50, 295);
    s.axis.limits = {true, 30, 300};
    s.start();
    for (int i = 0; i < 10; i++) {
        s.axis.jog(1);
        s.run(0.3f);
    }
    CHECK(fabsf(s.axis.target() - 300) < 1e-3f);
    s.run(2);
    CHECK(s.plant.angle() < 300.2f);
    // A target, hold() and a disable each end a jog.
    s.axis.jog(-1);
    s.run(0.2f);
    s.axis.setTarget(200);
    s.run(0.6f);
    CHECK(fabsf(s.axis.target() - 200) < 1e-3f);
    s.axis.jog(-1);
    s.axis.hold();
    s.run(0.3f);
    float held = s.axis.target();
    s.run(0.3f);
    CHECK_EQ(s.axis.target(), held);
    s.axis.jog(-1);
    s.axis.enable(false);
    s.axis.enable(true);
    s.run(0.5f);
    float here = s.axis.target();
    s.run(0.3f);
    CHECK_EQ(s.axis.target(), here);
}

TEST(axis_stops_a_hand_driven_motor) {
    Sim s(50, 0);
    s.run(0.1f);
    s.axis.setManualOutput(0.5f);
    s.run(0.2f);
    s.axis.hold();  // the tree's stop
    s.run(0.1f);
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
    CHECK_EQ(s.axis.output(), 0.0f);
    // stopAll() too, though the axis is already disabled.
    s.axis.setManualOutput(-0.5f);
    s.run(0.2f);
    s.motion.stopAll();
    s.run(0.1f);
    CHECK_EQ(s.plant.drivenDuty(), 0.0f);
}
