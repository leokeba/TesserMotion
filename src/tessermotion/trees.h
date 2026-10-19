#pragma once

// The parts' TesserAPI trees (docs/DESIGN.md section 9), found by
// argument-dependent lookup: api.mount("azimuth", azimuth).

#include <string>

#include "tesser/node.h"
#include "tessermotion/axis.h"
#include "tessermotion/motion.h"
#include "tessermotion/nmea_gps.h"
#include "tessermotion/tilt.h"
#include "tessermotion/tmc2209.h"
#include "tessermotion/tmc5160.h"

namespace tessermotion {

void describe(tesser::Object& o, Axis& axis);
void describe(tesser::Object& o, Motion& motion);
void describe(tesser::Object& o, Tilt& tilt);
void describe(tesser::Object& o, NmeaGps& gps);
void describe(tesser::Object& o, Tmc5160& driver);
void describe(tesser::Object& o, Tmc2209& driver);

// A Trinamic chip's register map as JSON, as the trees' `map` file serves it
// (docs/DESIGN.md section 4.8).
std::string tmcMapJson(const TmcMap& map);

}  // namespace tessermotion
