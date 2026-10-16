// Sensors: the NMEA parser, NmeaGps, the ADXL345 and Tilt.
#pragma once

#include <string>

#include "sim.h"

using namespace tessermotion;

namespace {

Nmea::Sentence feedLine(Nmea& p, const std::string& line) {
    Nmea::Sentence last = Nmea::Sentence::None;
    for (char c : line) {
        Nmea::Sentence s = p.feed(c);
        if (s != Nmea::Sentence::None) last = s;
    }
    return last;
}

// Classic examples, and a u-blox NEO-M8 style pair (GN talker, mode field).
const char* kRmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
const char* kGga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
const char* kRmcSouthWest = "$GNRMC,083559.00,A,3342.06123,S,07033.94012,W,0.004,,161026,,,A*6C\r\n";
const char* kGgaSouthWest = "$GNGGA,083559.00,3342.06123,S,07033.94012,W,2,12,0.71,512.3,M,32.1,M,,*45\r\n";
const char* kGgaNoFix = "$GPGGA,083600.00,,,,,0,00,99.99,,,,,,*6B\r\n";
const char* kRmcVoid = "$GPRMC,083600.00,V,,,,,,,161026,,,N*72\r\n";
const char* kGsv = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n";

bool near(double a, double b, double eps) { return fabs(a - b) < eps; }

// A Motion with one part, ticked at 50 Hz on the fake clock (which
// takeTime() reads).
struct SensorRig {
    Motion motion;
    explicit SensorRig(Part& part) {
        fake::reset();
        fake::now = 1000000;
        motion.add(part);
        motion.begin();
    }
    void tick() {
        fake::now += 20000;
        motion.step(fake::now);
    }
    void run(float seconds) {
        for (int i = 0; i < static_cast<int>(seconds * 50 + 0.5f); i++) tick();
    }
};

void adxl345Device(fake::I2c& bus, int16_t x, int16_t y, int16_t z, uint8_t addr = 0x53) {
    auto& r = bus.regs[addr];
    r[0x00] = 0xE5;
    int16_t v[3] = {x, y, z};
    for (int i = 0; i < 3; i++) {
        r[static_cast<uint8_t>(0x32 + 2 * i)] = static_cast<uint8_t>(v[i] & 0xFF);
        r[static_cast<uint8_t>(0x33 + 2 * i)] = static_cast<uint8_t>((v[i] >> 8) & 0xFF);
    }
}

constexpr float kG = 256 * 0.0039f * 9.80665f;  // 256 LSB, about 1 g

}  // namespace

TEST(nmea_epoch) {
    CHECK_EQ(Nmea::epochMs(1970, 1, 1, 0), int64_t(0));
    CHECK_EQ(Nmea::epochMs(2000, 3, 1, 0), int64_t(951868800) * 1000);
    CHECK_EQ(Nmea::epochMs(2024, 2, 29, 0), int64_t(1709164800) * 1000);
    CHECK_EQ(Nmea::epochMs(2024, 12, 28, 86399500), int64_t(1735430399500));
}

TEST(nmea_rmc_and_gga) {
    Nmea p;
    CHECK(feedLine(p, kRmc) == Nmea::Sentence::Rmc);
    CHECK(p.rmc().valid && p.rmc().hasDate && p.rmc().hasPosition);
    CHECK_EQ(p.rmc().epochMs, int64_t(764426119) * 1000);
    CHECK(near(p.rmc().latitude, 48 + 7.038 / 60, 1e-9));
    CHECK(near(p.rmc().longitude, 11 + 31.0 / 60, 1e-9));
    CHECK(feedLine(p, kGga) == Nmea::Sentence::Gga);
    CHECK_EQ(p.gga().quality, uint8_t(1));
    CHECK_EQ(p.gga().satellites, uint8_t(8));
    CHECK(fabsf(p.gga().hdop - 0.9f) < 1e-6f);
    CHECK(fabsf(p.gga().altitude - 545.4f) < 1e-4f);
    CHECK(p.gga().hasPosition);
    CHECK(near(p.gga().latitude, 48 + 7.038 / 60, 1e-9));
    // Any talker, the southern and western hemispheres, fractional seconds.
    CHECK(feedLine(p, kRmcSouthWest) == Nmea::Sentence::Rmc);
    CHECK(near(p.rmc().latitude, -(33 + 42.06123 / 60), 1e-9));
    CHECK(near(p.rmc().longitude, -(70 + 33.94012 / 60), 1e-9));
    CHECK_EQ(p.rmc().epochMs, int64_t(1792139759) * 1000);
    CHECK(feedLine(p, kGgaSouthWest) == Nmea::Sentence::Gga);
    CHECK_EQ(p.gga().quality, uint8_t(2));
    CHECK_EQ(p.gga().satellites, uint8_t(12));
    CHECK(feedLine(p, "$GNRMC,235959.50,A,4807.038,N,01131.000,E,0.0,,281224,,,A*67\n") == Nmea::Sentence::Rmc);
    CHECK_EQ(p.rmc().epochMs, int64_t(1735430399500));
    CHECK_EQ(p.errors(), 0u);
}

TEST(nmea_no_fix) {
    Nmea p;
    feedLine(p, kGga);
    CHECK(feedLine(p, kGgaNoFix) == Nmea::Sentence::Gga);
    CHECK_EQ(p.gga().quality, uint8_t(0));
    CHECK_EQ(p.gga().satellites, uint8_t(0));
    CHECK(!p.gga().hasPosition);
    // A void RMC carries a date from the receiver's clock, which isn't trusted.
    CHECK(feedLine(p, kRmcVoid) == Nmea::Sentence::Rmc);
    CHECK(!p.rmc().valid && !p.rmc().hasDate && !p.rmc().hasPosition);
    CHECK_EQ(p.errors(), 0u);
}

TEST(nmea_drops_bad_lines) {
    Nmea p;
    // Joining in the middle of a line: ignored until a '$'.
    CHECK(feedLine(p, "0.9,545.4,M,46.9,M,,*47\r\n") == Nmea::Sentence::None);
    CHECK_EQ(p.errors(), 0u);
    // A wrong checksum, a missing one, a malformed field.
    CHECK(feedLine(p, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6B\r\n") ==
          Nmea::Sentence::Error);
    CHECK(feedLine(p, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W\r\n") ==
          Nmea::Sentence::Error);
    CHECK(feedLine(p, "$GPGGA,123519,48O7.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*38\r\n") ==
          Nmea::Sentence::Error);
    CHECK_EQ(p.errors(), 3u);
    CHECK(!p.rmc().valid);  // nothing was taken from them
    CHECK_EQ(p.gga().quality, uint8_t(0));
    // Cut short by the next sentence, which still parses.
    CHECK(feedLine(p, "$GPRMC,1235") == Nmea::Sentence::None);
    CHECK(feedLine(p, kGga) == Nmea::Sentence::Gga);
    CHECK_EQ(p.errors(), 4u);
    // Too long.
    CHECK(feedLine(p, "$GPTXT," + std::string(120, 'x') + "\r\n") == Nmea::Sentence::Error);
    CHECK_EQ(p.errors(), 5u);
    // Other sentences are ignored, not errors.
    CHECK(feedLine(p, kGsv) == Nmea::Sentence::Other);
    CHECK_EQ(p.errors(), 5u);
}

TEST(gps_reports_the_fix) {
    fake::Uart uart;
    NmeaGps gps(uart);
    SensorRig r(gps);
    CHECK_EQ(uart.begins, 1u);
    NmeaGps::Fix f = gps.fix();
    CHECK(f.quality == GpsFix::None);
    CHECK_EQ(f.ageMs, -1);
    CHECK_EQ(gps.utc(), "");
    // Sentences split across ticks.
    std::string both = std::string(kRmcSouthWest) + kGgaSouthWest;
    uart.chunk = 10;
    uart.send(both);
    r.tick();
    CHECK(gps.fix().quality == GpsFix::None);
    uart.chunk = 64;
    r.run(0.1f);
    f = gps.fix();
    CHECK(f.quality == GpsFix::Dgps);
    CHECK_EQ(f.satellites, uint8_t(12));
    CHECK(near(f.latitude, -(33 + 42.06123 / 60), 1e-9));
    CHECK(fabsf(f.altitude - 512.3f) < 1e-3f);
    CHECK(f.ageMs >= 0 && f.ageMs < 100);
    CHECK_EQ(gps.utc(), "2026-10-16T08:35:59Z");
    CHECK_EQ(gps.errors(), 0u);
    // No fix: the last position is kept, ageing.
    uart.send(kGgaNoFix);
    r.run(1);
    f = gps.fix();
    CHECK(f.quality == GpsFix::None);
    CHECK(near(f.latitude, -(33 + 42.06123 / 60), 1e-9));
    CHECK(f.ageMs >= 1000);
    // Silent for kStaleMs: the fix is none.
    uart.send(kGga);
    r.tick();
    CHECK(gps.fix().quality == GpsFix::Gps);
    r.run(2.9f);
    CHECK(gps.fix().quality == GpsFix::Gps);
    r.run(0.2f);
    CHECK(gps.fix().quality == GpsFix::None);
    CHECK_EQ(gps.fix().satellites, uint8_t(0));
    uart.send("$GPGGA,bad*00\r\n");
    r.tick();
    CHECK_EQ(gps.errors(), 1u);
}

TEST(gps_hands_the_time_over_once) {
    fake::Uart uart;
    NmeaGps gps(uart);
    SensorRig r(gps);
    int64_t t = 0;
    CHECK(!gps.takeTime(t));
    uart.send(kRmcVoid);
    r.tick();
    CHECK(!gps.takeTime(t));  // no fix, no time
    uart.send(kRmcSouthWest);
    r.tick();
    // Advanced by the time since the sentence came.
    fake::now += 250000;
    CHECK(gps.takeTime(t));
    CHECK_EQ(t, int64_t(1792139759) * 1000 + 250);
    CHECK(!gps.takeTime(t));  // once
    uart.send(kRmc);
    r.tick();
    CHECK(gps.takeTime(t));
    CHECK_EQ(t, int64_t(764426119) * 1000);
}

TEST(gps_without_its_uart) {
    fake::Uart uart;
    uart.ok = false;
    NmeaGps gps(uart);
    SensorRig r(gps);
    uart.send(kGga);
    r.run(0.1f);
    CHECK_EQ(uart.reads, 0u);
    CHECK(gps.fix().quality == GpsFix::None);
}

TEST(adxl345_configures_and_reads) {
    fake::I2c bus;
    adxl345Device(bus, 256, -512, 1000);
    Adxl345 accel(bus);
    CHECK(accel.begin());
    CHECK_EQ(bus.regs[0x53][0x2C], uint8_t(0x0A));  // 100 Hz
    CHECK_EQ(bus.regs[0x53][0x31], uint8_t(0x0B));  // full resolution, ±16 g
    CHECK_EQ(bus.regs[0x53][0x2D], uint8_t(0x08));  // measure
    float x, y, z;
    CHECK(accel.read(x, y, z));
    CHECK_EQ(bus.lastReg, uint8_t(0x32));
    CHECK_EQ(bus.lastLen, size_t(6));
    CHECK(fabsf(x - kG) < 1e-4f);
    CHECK(fabsf(y + 2 * kG) < 1e-4f);
    CHECK(fabsf(z - 1000 * 0.0039f * 9.80665f) < 1e-3f);
    // At the other address.
    fake::I2c bus2;
    adxl345Device(bus2, 0, 0, 256, 0x1D);
    Adxl345 other(bus2, 0x1D);
    CHECK(other.begin());
}

TEST(adxl345_absent_or_wrong) {
    fake::I2c bus;
    Adxl345 accel(bus);
    CHECK(!accel.begin());  // NACK
    bus.regs[0x53][0x00] = 0x42;
    CHECK(!accel.begin());  // not an ADXL345
    CHECK_EQ(bus.writes, 0u);
    CHECK_EQ(bus.recovers, 0u);
}

TEST(tilt_averages) {
    fake::I2c bus;
    adxl345Device(bus, 0, 0, 256);
    Adxl345 accel(bus);
    Tilt tilt(accel);
    tilt.samples = 4;
    tilt.every = 5;
    SensorRig r(tilt);
    CHECK(tilt.present());
    uint32_t reads = bus.reads;
    r.run(1);
    CHECK_EQ(bus.reads - reads, 10u);  // every 5 ticks
    Tilt::Reading t = tilt.reading();
    CHECK(fabsf(t.z - kG) < 1e-4f);
    CHECK(fabsf(t.pitch) < 1e-3f && fabsf(t.roll) < 1e-3f);
    // Tilted 90° about y: x reads −1 g. The mean moves over `samples` reads.
    adxl345Device(bus, -256, 0, 0);
    for (int i = 0; i < 5; i++) r.tick();
    t = tilt.reading();
    CHECK(fabsf(t.x + kG / 4) < 1e-4f);
    CHECK(fabsf(t.z - 3 * kG / 4) < 1e-4f);
    r.run(0.4f);
    t = tilt.reading();
    CHECK(fabsf(t.x + kG) < 1e-4f);
    CHECK(fabsf(t.pitch - 90) < 1e-3f);
    // Rolled 45°.
    adxl345Device(bus, 0, 181, 181);
    r.run(0.4f);
    t = tilt.reading();
    CHECK(fabsf(t.roll - 45) < 1e-3f);
    CHECK(fabsf(t.pitch) < 1e-3f);
}

TEST(tilt_absent_sensor_leaves_the_bus_alone) {
    // An MT6701 and a missing ADXL345 on one bus: the accelerometer is
    // probed every 2 s, never read, and never recovers the bus.
    fake::I2c bus;
    bus.regs[0x06][0x03] = 0x40;
    Mt6701 encoder(bus);
    Adxl345 accel(bus);
    Tilt tilt(accel);
    struct Reader : Part {
        Mt6701& e;
        int ok = 0;
        explicit Reader(Mt6701& m) : e(m) {}
        void sense(uint64_t) override {
            float d;
            if (e.read(d)) ok++;
        }
        void update(uint64_t) override {}
    } reader(encoder);
    SensorRig r(tilt);
    r.motion.add(reader);
    CHECK(!tilt.present());
    uint32_t probes = bus.reads;
    r.run(10);
    CHECK_EQ(reader.ok, 500);
    CHECK_EQ(bus.recovers, 0u);
    CHECK_EQ(bus.reads - probes, 500u + 4u);  // the encoder's, and a probe every 2 s
    // Plugged in: found at the next probe.
    adxl345Device(bus, 0, 0, 256);
    r.run(2);
    CHECK(tilt.present());
    r.run(0.2f);
    CHECK(fabsf(tilt.reading().z - kG) < 1e-4f);
}

TEST(tilt_loses_and_finds_its_sensor) {
    fake::I2c bus;
    adxl345Device(bus, 0, 0, 256);
    Adxl345 accel(bus);
    Tilt tilt(accel);
    tilt.every = 1;
    SensorRig r(tilt);
    r.run(0.2f);
    CHECK(tilt.present());
    bus.stuck = true;
    r.tick();
    r.tick();
    CHECK(tilt.present());  // two failures
    r.tick();
    CHECK(!tilt.present());
    CHECK_EQ(bus.recovers, 1u);  // the third recovered the bus
    // Probed again after 2 s, and averaging anew.
    adxl345Device(bus, 0, 256, 0);
    r.run(1.9f);
    CHECK(!tilt.present());
    r.run(0.2f);
    CHECK(tilt.present());
    r.run(0.1f);
    CHECK(fabsf(tilt.reading().y - kG) < 1e-4f);
    CHECK(fabsf(tilt.reading().z) < 1e-4f);
}
