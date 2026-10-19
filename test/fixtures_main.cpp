// Writes the parts' schemas, with their hints, as golden fixtures
// (docs/DESIGN.md section 11): ./tessermotion_fixtures <dir>
#include <stdio.h>
#include <sys/stat.h>

#include <ArduinoJson.h>

#include <string>

#include "TesserAPI.h"
#include "TesserMotion.h"
#include "tessermotion/tmc/tmc2209_map.h"
#include "tessermotion/tmc/tmc5160_map.h"
#include "fake_hw.h"
#include "fake_tmc.h"
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

bool writeText(const std::string& text, const char* dir, const char* name) {
    std::string file = std::string(dir) + "/" + name;
    FILE* f = fopen(file.c_str(), "w");
    if (!f) return false;
    fputs(text.c_str(), f);
    fputc('\n', f);
    fclose(f);
    printf("wrote %s (%zu bytes)\n", file.c_str(), text.size());
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
    fake::I2c bus;
    tessermotion::Adxl345 accel(bus);
    tessermotion::Tilt tilt(accel);
    fake::Uart uart;
    tessermotion::NmeaGps gps(uart);
    motion.add(tilt);
    motion.add(gps);
    fake::Spi spi;
    fake::Tmc5160 chip5160;
    chip5160.attach(spi, 5);
    tessermotion::Tmc5160 stepper(spi, tessermotion::Tmc5160Config{5});
    fake::Uart tmcUart;
    fake::Tmc2209 chip2209;
    chip2209.attach(tmcUart);
    tessermotion::Tmc2209 small(tmcUart);
    motion.add(stepper);
    motion.add(small);
    tesser::Api api;
    api.mount("azimuth", axis).label("Azimuth");
    api.mount("motion", motion);
    api.mount("tilt", tilt).label("Tilt");
    api.mount("gps", gps).label("GPS");
    api.mount("stepper", stepper).label("Stepper");
    api.mount("small", small).label("Small stepper");
    if (tesser::declarationErrors() != 0) return 1;
    return write(api, "/azimuth", dir, "axis.json") && write(api, "/motion", dir, "motion.json") &&
                   write(api, "/tilt", dir, "tilt.json") && write(api, "/gps", dir, "gps.json") &&
                   write(api, "/stepper", dir, "tmc5160.json") && write(api, "/small", dir, "tmc2209.json") &&
                   writeText(tessermotion::tmcMapJson(tessermotion::tmc5160::kMap), dir, "tmc5160_map.json") &&
                   writeText(tessermotion::tmcMapJson(tessermotion::tmc2209::kMap), dir, "tmc2209_map.json")
               ? 0
               : 1;
}
