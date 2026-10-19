// The trees (docs/DESIGN.md section 9), through TesserAPI's request handler.
#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <ArduinoJson.h>

#include "TesserAPI.h"
#include "accel_cases.h"  // its poses
#include "sim.h"

using namespace tessermotion;

namespace {

struct TreeRig : Sim {
    tesser::Api api;
    fake::I2c bus;
    Adxl345 accel{bus};
    Tilt tilt{accel};
    fake::Uart uart;
    NmeaGps gps{uart};

    TreeRig() : Sim(50, 100) {
        // Added after Sim's motion.begin(), so begun here.
        motion.add(tilt);
        motion.add(gps);
        tilt.begin();
        gps.begin();
        api.mount("azimuth", axis);
        api.mount("tilt", tilt);
        api.mount("gps", gps);
        api.object("sys").mount("motion", motion);
    }

    struct Reply {
        tesser::Status status;
        std::string body;
    };
    Reply request(tesser::Op op, std::string_view path, std::string_view body = {},
                  tesser::View view = tesser::View::Value) {
        JsonDocument doc;
        if (!body.empty()) deserializeJson(doc, body.data(), body.size());
        tesser::Request req;
        req.op = op;
        req.path = path;
        req.body = doc.as<JsonVariantConst>();
        req.query.view = view;
        req.client.authenticated = true;
        tesser::StringReply reply;
        api.handle(req, reply);
        return {reply.status, reply.body};
    }
    Reply get(std::string_view path) { return request(tesser::Op::Get, path); }
    Reply set(std::string_view path, std::string_view body) { return request(tesser::Op::Set, path, body); }
    float number(std::string_view path) {
        JsonDocument doc;
        deserializeJson(doc, get(path).body);
        return doc.as<float>();
    }
};

}  // namespace

TEST(tree_axis_values) {
    TreeRig r;
    CHECK_EQ(tesser::declarationErrors(), 0);
    r.run(0.1f);
    CHECK(fabsf(r.number("/azimuth/position") - 100) < 0.05f);
    CHECK_EQ(r.get("/azimuth/state").body, "\"disabled\"");
    // A target is refused until the axis runs.
    CHECK(r.set("/azimuth/target", "120").status == tesser::Status::NotAllowed);
    CHECK(r.set("/azimuth/enabled", "true").status == tesser::Status::Ok);
    r.run(0.1f);
    CHECK_EQ(r.get("/azimuth/state").body, "\"holding\"");
    CHECK(r.set("/azimuth/target", "120").status == tesser::Status::Ok);
    CHECK(r.set("/azimuth/target", "400").status == tesser::Status::InvalidValue);
    r.run(5);
    CHECK(fabsf(r.number("/azimuth/position") - 120) < 0.15f);
    CHECK(fabsf(r.number("/azimuth/sensor/raw") - 120) < 0.15f);
    // Settings are the axis's own fields.
    CHECK(r.set("/azimuth/tuning", "{\"kp\": 0.08, \"maxSpeed\": 12}").status == tesser::Status::Ok);
    CHECK(fabsf(r.axis.tuning.kp - 0.08f) < 1e-6f);
    CHECK_EQ(r.axis.tuning.maxSpeed, 12.0f);
    CHECK(r.set("/azimuth/sensor/config/offset", "10").status == tesser::Status::Ok);
    CHECK(fabsf(r.number("/azimuth/position") - 130) < 0.15f);
    CHECK(r.set("/azimuth/limits", "{\"enabled\": true, \"begin\": 30, \"end\": 300}").status == tesser::Status::Ok);
    CHECK(r.axis.limits.enabled);
    CHECK(r.set("/azimuth/stop", "").status == tesser::Status::Ok);
}

TEST(tree_axis_manual_output) {
    TreeRig r;
    CHECK(r.set("/azimuth/motor/output", "0.5").status == tesser::Status::Ok);
    r.run(0.5f);
    CHECK(r.plant.drivenDuty() > 0.5f);
    CHECK(fabsf(r.number("/azimuth/motor/output") - 0.5f) < 1e-6f);
    CHECK_EQ(r.get("/azimuth/motor/duty").body, "58");
    CHECK(r.set("/azimuth/motor/config/minOutput", "0.2").status == tesser::Status::Ok);
    CHECK_EQ(r.plant.minOutput, 0.2f);
    CHECK(r.set("/azimuth/enabled", "true").status == tesser::Status::Ok);
    CHECK(r.set("/azimuth/motor/output", "0.5").status == tesser::Status::NotAllowed);
}

TEST(tree_axis_claimed_target_is_busy) {
    TreeRig r;
    r.axis.enable(true);
    r.run(0.1f);
    r.axis.claim("tracking");
    auto reply = r.set("/azimuth/target", "120");
    CHECK(reply.status == tesser::Status::Busy);
    CHECK(reply.body.find("driven by tracking") != std::string::npos);
    CHECK(r.axis.setTarget(120));  // code isn't refused
    reply = r.set("/azimuth/jogPlus", "true");
    CHECK(reply.status == tesser::Status::Busy);
    CHECK(reply.body.find("driven by tracking") != std::string::npos);
    r.axis.release();
    CHECK(r.set("/azimuth/target", "130").status == tesser::Status::Ok);
    CHECK(r.set("/azimuth/jogPlus", "true").status == tesser::Status::Ok);
    CHECK(r.set("/azimuth/jogPlus", "false").status == tesser::Status::Ok);
    r.axis.enable(false);
    reply = r.set("/azimuth/jogMinus", "true");
    CHECK(reply.status == tesser::Status::NotAllowed);
    CHECK(reply.body.find("isn't running") != std::string::npos);
}

TEST(tree_motion) {
    TreeRig r;
    r.axis.enable(true);
    r.run(0.1f);
    CHECK(fabsf(r.number("/sys/motion/period") - 20) < 1e-3f);
    CHECK(r.set("/sys/motion/stopAll", "").status == tesser::Status::Ok);
    CHECK(!r.axis.enabled());
}

TEST(tree_tilt) {
    TreeRig r;
    CHECK_EQ(r.get("/tilt/present").body, "false");
    r.bus.regs[0x53][0x00] = 0xE5;
    r.bus.regs[0x53][0x37] = 0x01;  // z: 256 LSB
    r.run(2.2f);                     // the next probe
    CHECK_EQ(r.get("/tilt/present").body, "true");
    CHECK(fabsf(r.number("/tilt/z") - 9.79f) < 0.01f);
    CHECK(fabsf(r.number("/tilt/pitch")) < 1e-3f);
    CHECK(r.set("/tilt/samples", "16").status == tesser::Status::Ok);
    CHECK_EQ(r.tilt.samples, 16u);
    CHECK(r.set("/tilt/samples", "0").status == tesser::Status::InvalidValue);
    CHECK(r.set("/tilt/samples", "33").status == tesser::Status::InvalidValue);
}

TEST(tree_tilt_correction) {
    TreeRig r;
    const std::string c = "/tilt/correction";
    CHECK(r.set(c + "/record", "").status == tesser::Status::NotAllowed);  // no sensor
    r.bus.regs[0x53][0x00] = 0xE5;
    r.bus.regs[0x53][0x37] = 0x01;  // z: 256 LSB
    r.run(3.2f);                     // found, and a full average
    CHECK(r.set(c + "/record", "").status == tesser::Status::Ok);
    CHECK_EQ(r.tilt.calibration.points.size(), size_t(1));
    CHECK(fabsf(r.number(c + "/points/0/z") - 9.79f) < 0.01f);
    CHECK_EQ(r.get(c + "/residuals/points").body, "1");
    CHECK(r.set(c + "/fit", "").status == tesser::Status::InvalidValue);  // too few
    // Points written whole, from elsewhere.
    std::string points = "[";
    for (const AccelPoint& p : accelPoses(14, accelTruth(), 0.02f)) {
        points += (points.size() > 1 ? "," : "") + std::string("{\"x\":") + std::to_string(p.x) +
                  ",\"y\":" + std::to_string(p.y) + ",\"z\":" + std::to_string(p.z) + "}";
    }
    CHECK(r.set(c + "/points", points + "]").status == tesser::Status::Ok);
    CHECK(r.number(c + "/residuals/rms") > 0.5f);
    CHECK(r.set(c + "/fit", "").status == tesser::Status::Ok);
    CHECK(fabsf(r.number(c + "/bias/2") + 1.3f) < 0.02f);
    CHECK(fabsf(r.number(c + "/scale/2") - 1.06f) < 0.003f);
    CHECK(r.number(c + "/residuals/rms") < 0.02f);
    r.run(0.2f);
    CHECK(fabsf(r.number("/tilt/z") - 1.06f * (9.79f + 1.3f)) < 0.05f);  // corrected
    CHECK(r.set(c + "/bias/0", "0").status != tesser::Status::Ok);    // only fit writes it
    // Saved: the bias, the scale and the points.
    tesser::MemoryStorage storage;
    r.api.persistence(storage);
    std::string state = r.api.persistedState();
    CHECK(state.find("\"correction\":{\"points\":[{\"x\":") != std::string::npos);
    CHECK(state.find("\"bias\":[0.2") != std::string::npos);
    CHECK(state.find("\"scale\":[1.0") != std::string::npos);
    CHECK(r.set(c + "/clear", "").status == tesser::Status::Ok);
    CHECK(r.tilt.correction.scale[2] == 1 && r.tilt.correction.bias[2] == 0);
    CHECK_EQ(r.tilt.calibration.points.size(), size_t(14));
}

TEST(tree_gps) {
    TreeRig r;
    CHECK_EQ(r.get("/gps/fix").body, "\"none\"");
    CHECK_EQ(r.get("/gps/age").body, "-1");
    CHECK_EQ(r.get("/gps/utc").body, "\"\"");
    r.uart.send("$GNRMC,083559.00,A,3342.06123,S,07033.94012,W,0.004,,161026,,,A*6C\r\n"
                "$GNGGA,083559.00,3342.06123,S,07033.94012,W,2,12,0.71,512.3,M,32.1,M,,*45\r\n");
    r.run(0.1f);
    CHECK_EQ(r.get("/gps/fix").body, "\"dgps\"");
    CHECK_EQ(r.get("/gps/satellites").body, "12");
    CHECK(fabsf(r.number("/gps/latitude") + 33.701020f) < 1e-5f);
    CHECK(fabsf(r.number("/gps/longitude") + 70.565669f) < 1e-5f);
    CHECK_EQ(r.get("/gps/utc").body, "\"2026-10-16T08:35:59Z\"");
    CHECK(r.set("/gps/fix", "\"gps\"").status != tesser::Status::Ok);  // read-only
}

TEST(tree_axis_correction) {
    TreeRig r;
    const std::string c = "/azimuth/sensor/correction";
    r.run(0.1f);
    // Recorded at the reading (100°), referenced in the output frame.
    CHECK(r.set(c + "/record", "110").status == tesser::Status::Ok);
    CHECK_EQ(r.axis.calibration.points.size(), size_t(1));
    CHECK(fabsf(r.number(c + "/points/0/raw") - 100) < 0.05f);
    CHECK(fabsf(r.number(c + "/points/0/reference") - 110) < 1e-4f);
    CHECK_EQ(r.get(c + "/residuals/points").body, "1");
    CHECK(fabsf(r.number(c + "/residuals/zero") - 10) < 0.05f);
    // Too few points for the harmonics asked.
    CHECK(r.set(c + "/harmonics", "2").status == tesser::Status::Ok);
    CHECK(r.set(c + "/fit", "").status == tesser::Status::InvalidValue);
    // Points written whole, from elsewhere: a 1° first harmonic.
    std::string points = "[";
    for (int i = 0; i < 12; i++) {
        float raw = 30.0f * static_cast<float>(i);
        float ref = wrap360(raw + sinf(raw * 0.017453292f));
        points += (i ? "," : "") + std::string("{\"raw\":") + std::to_string(raw) + ",\"reference\":" + std::to_string(ref) + "}";
    }
    CHECK(r.set(c + "/points", points + "]").status == tesser::Status::Ok);
    CHECK(r.number(c + "/residuals/max") > 0.9f);
    float before = r.number("/azimuth/position");
    CHECK(r.set(c + "/fit", "").status == tesser::Status::Ok);
    CHECK(fabsf(r.number(c + "/sin/0") - 1) < 0.01f);
    CHECK(r.number(c + "/residuals/max") < 0.01f);
    CHECK(fabsf(r.number(c + "/residuals/coverage") - 330) < 0.01f);
    CHECK(fabsf(r.number("/azimuth/position") - before - sinf(100 * 0.017453292f)) < 0.01f);
    CHECK(r.set(c + "/sin/0", "0").status != tesser::Status::Ok);  // only fit writes them
    // Saved: the coefficients, the points and the harmonics.
    tesser::MemoryStorage storage;
    r.api.persistence(storage);
    std::string state = r.api.persistedState();
    CHECK(state.find("\"correction\":{\"harmonics\":2,\"points\":[{\"raw\":0,") != std::string::npos);
    CHECK(state.find("\"sin\":[0.99") != std::string::npos);
    CHECK(state.find("residuals") == std::string::npos);
    CHECK(r.set(c + "/clear", "").status == tesser::Status::Ok);
    CHECK_EQ(r.axis.correction.harmonics(), 0);
    CHECK_EQ(r.axis.calibration.points.size(), size_t(12));
    CHECK(fabsf(r.number("/azimuth/position") - before) < 0.05f);
}

TEST(tree_persists_settings_only) {
    TreeRig r;
    tesser::MemoryStorage storage;
    r.api.persistence(storage);
    r.axis.tuning.kp = 0.07f;
    r.axis.limits = {true, 20, 200};
    std::string state = r.api.persistedState();
    CHECK(state.find("\"kp\":0.07") != std::string::npos);
    CHECK(state.find("\"limits\":{\"enabled\":true,\"begin\":20,\"end\":200}") != std::string::npos);
    CHECK(state.find("\"minOutput\"") != std::string::npos);
    CHECK(state.find("\"offset\"") != std::string::npos);
    CHECK(state.find("position") == std::string::npos);
    CHECK(state.find("enabled\":false") == std::string::npos);  // axes boot disabled
    CHECK(state.find("period") == std::string::npos);
    CHECK(state.find("\"tilt\":{\"samples\":8,\"correction\":{\"points\":[],\"bias\":[0,0,0],\"scale\":[1,1,1]}}") !=
          std::string::npos);
    CHECK(state.find("gps") == std::string::npos);
}

TEST(tree_watches_what_code_changes) {
    // Values the control loop or code change without a tree write must be
    // watched, or subscribers keep stale values: everything readable but
    // the settings (written only through the tree) and tickMax (reset on read).
    TreeRig r;
    std::vector<std::string> unwatched;
    std::function<void(const tesser::Object&, const std::string&, bool)> walk = [&](const tesser::Object& o,
                                                                                    const std::string& path,
                                                                                    bool setting) {
        for (const tesser::Node* c = o.first(); c; c = c->next()) {
            std::string p = path + "/" + c->name();
            bool s = setting || c->persisted();
            if (c->type() == tesser::NodeType::Object) {
                walk(static_cast<const tesser::Object&>(*c), p, s);
            } else if (c->type() == tesser::NodeType::Value && !c->watched() && !s) {
                unwatched.push_back(p);
            }
        }
    };
    walk(r.api, "", false);
    CHECK_EQ(unwatched.size(), size_t(1));
    CHECK(!unwatched.empty() && unwatched[0] == "/sys/motion/tickMax");
}
