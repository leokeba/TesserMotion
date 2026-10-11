// The trees (docs/DESIGN.md section 9), through TesserAPI's request handler.
#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <ArduinoJson.h>

#include "TesserAPI.h"
#include "sim.h"

using namespace tessermotion;

namespace {

struct TreeRig : Sim {
    tesser::Api api;

    TreeRig() : Sim(50, 100) {
        api.mount("azimuth", axis);
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
    CHECK(r.set("/azimuth/jogPlus", "true").status == tesser::Status::Busy);
    r.axis.release();
    CHECK(r.set("/azimuth/target", "130").status == tesser::Status::Ok);
    CHECK(r.set("/azimuth/jogPlus", "true").status == tesser::Status::Ok);
    CHECK(r.set("/azimuth/jogPlus", "false").status == tesser::Status::Ok);
    r.axis.enable(false);
    CHECK(r.set("/azimuth/jogMinus", "true").status == tesser::Status::NotAllowed);
}

TEST(tree_motion) {
    TreeRig r;
    r.axis.enable(true);
    r.run(0.1f);
    CHECK(fabsf(r.number("/sys/motion/period") - 20) < 1e-3f);
    CHECK(r.set("/sys/motion/stopAll", "").status == tesser::Status::Ok);
    CHECK(!r.axis.enabled());
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
