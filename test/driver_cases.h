// Drivers: the DC motor's output mapping, the TB6612, the MT6701.
#pragma once

#include "sim.h"

using namespace tessermotion;

namespace {

struct RecordingMotor : DcMotor {
    bool forward = true;
    float driven = -1;
    bool begin() override { return true; }
    void drive(bool f, float d) override {
        forward = f;
        driven = d;
    }
};

}  // namespace

TEST(motor_output_mapping) {
    RecordingMotor m;
    m.output(0.5f);
    CHECK(fabsf(m.driven - (0.5f * 0.85f + 0.15f)) < 1e-6f);
    CHECK(m.forward);
    m.output(-1);
    CHECK(fabsf(m.driven - 1) < 1e-6f);
    CHECK(!m.forward);
    // Zero: no duty, the direction kept.
    m.output(0);
    CHECK_EQ(m.driven, 0.0f);
    CHECK(!m.forward);
    m.output(0.0005f);
    CHECK_EQ(m.driven, 0.0f);
    // Out of range and NaN.
    m.output(3);
    CHECK(fabsf(m.driven - 1) < 1e-6f);
    CHECK_EQ(m.lastOutput(), 1.0f);
    m.output(NAN);
    CHECK_EQ(m.driven, 0.0f);
    CHECK_EQ(m.lastOutput(), 0.0f);
    // Shape.
    m.shape = 2;
    m.minOutput = 0;
    m.output(0.5f);
    CHECK(fabsf(m.driven - 0.25f) < 1e-6f);
}

TEST(motor_invert) {
    RecordingMotor m;
    m.invert = true;
    m.output(0.5f);
    CHECK(!m.forward);
    m.output(-0.5f);
    CHECK(m.forward);
}

TEST(tb6612_truth_table) {
    fake::reset();
    Tb6612Standby standby(18);
    Tb6612Channel motor(standby, {16, 17, 5});
    CHECK(fake::gpio.writes == 0);  // constructors touch nothing
    CHECK(motor.begin());
    CHECK(fake::gpio.isOutput[17] && fake::gpio.isOutput[5]);
    CHECK_EQ(fake::pwm.channels.size(), size_t(1));
    CHECK_EQ(fake::pwm.channels[0].pin, 16);
    CHECK_EQ(fake::pwm.channels[0].hz, 15000u);
    CHECK(fake::gpio.level[18]);  // standby released after the channel's pins
    motor.output(0.5f);
    CHECK(fake::gpio.level[17] && !fake::gpio.level[5]);
    CHECK(fabsf(fake::pwm.dutyOn(16) - 0.575f) < 1e-6f);
    motor.output(-0.5f);
    CHECK(!fake::gpio.level[17] && fake::gpio.level[5]);
    // Duty 0 brakes: PWM low, the direction pins unchanged.
    motor.output(0);
    CHECK_EQ(fake::pwm.dutyOn(16), 0.0f);
    CHECK(!fake::gpio.level[17] && fake::gpio.level[5]);
    standby.enable(false);
    CHECK(!fake::gpio.level[18]);
}

TEST(tb6612_without_pwm) {
    fake::reset();
    fake::pwm.capacity = 0;
    Tb6612Standby standby(18);
    Tb6612Channel motor(standby, {16, 17, 5});
    CHECK(!motor.begin());
    CHECK(!fake::gpio.level[18]);  // the bridge stays in standby
    motor.output(1);               // harmless
}

TEST(mt6701_reads_the_angle) {
    fake::I2c bus;
    bus.regs[0x06][0x03] = 0xFF;
    bus.regs[0x06][0x04] = 0xFC;  // 16383 counts
    Mt6701 sensor(bus);
    CHECK(sensor.begin());
    float deg = -1;
    CHECK(sensor.read(deg));
    CHECK_EQ(bus.lastAddr, uint8_t(0x06));
    CHECK_EQ(bus.lastReg, uint8_t(0x03));
    CHECK_EQ(bus.lastLen, size_t(2));
    CHECK(fabsf(deg - 16383 * 360.0f / 16384) < 1e-4f);
    bus.regs[0x06][0x03] = 0x80;
    bus.regs[0x06][0x04] = 0x00;  // 8192 counts
    CHECK(sensor.read(deg));
    CHECK(fabsf(deg - 180) < 1e-4f);
    // 0° is a reading like any other.
    bus.regs[0x06][0x03] = 0;
    CHECK(sensor.read(deg));
    CHECK_EQ(deg, 0.0f);
}

TEST(mt6701_recovers_the_bus) {
    fake::I2c bus;
    bus.regs[0x06][0x03] = 0x40;
    Mt6701 sensor(bus);
    float deg;
    bus.failNext = 2;
    CHECK(!sensor.read(deg));
    CHECK(!sensor.read(deg));
    CHECK(sensor.read(deg));  // a success resets the count
    CHECK_EQ(bus.recovers, 0u);
    bus.stuck = true;
    CHECK(!sensor.read(deg));
    CHECK(!sensor.read(deg));
    CHECK(!sensor.read(deg));  // the third failure in a row recovers
    CHECK_EQ(bus.recovers, 1u);
    CHECK(sensor.read(deg));
    CHECK(fabsf(deg - 90) < 1e-4f);
}
