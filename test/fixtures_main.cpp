// Writes the parts' schemas, with their hints, as golden fixtures
// (docs/DESIGN.md section 11): ./tessermotion_fixtures <dir>
#include <stdio.h>
#include <sys/stat.h>

#include <ArduinoJson.h>

#include <string>

#include "TesserAPI.h"
#include "TesserMotion.h"
#include "fake_hw.h"
#include "plant.h"

namespace {

bool write(tesser::Api& api, const char* path, const char* dir, const char* name) {
    tesser::Request req;
    req.op = tesser::Op::Get;
    req.path = path;
    req.query.view = tesser::View::Schema;
    req.client.authenticated = true;
    tesser::StringReply reply;
    api.handle(req, reply);
    if (reply.status != tesser::Status::Ok) return false;
    std::string file = std::string(dir) + "/" + name;
    FILE* f = fopen(file.c_str(), "w");
    if (!f) return false;
    fputs(reply.body.c_str(), f);
    fputc('\n', f);
    fclose(f);
    printf("wrote %s (%zu bytes)\n", file.c_str(), reply.body.size());
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : "fixtures";
    mkdir(dir, 0755);
    Plant plant;
    tessermotion::Axis axis(plant, plant);
    tessermotion::Motion motion;
    motion.add(axis);
    tesser::Api api;
    api.mount("azimuth", axis).label("Azimuth");
    api.mount("motion", motion);
    if (tesser::declarationErrors() != 0) return 1;
    return write(api, "/azimuth", dir, "axis.json") && write(api, "/motion", dir, "motion.json") ? 0 : 1;
}
