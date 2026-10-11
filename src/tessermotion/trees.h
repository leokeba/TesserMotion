#pragma once

// The parts' TesserAPI trees (docs/DESIGN.md section 9), found by
// argument-dependent lookup: api.mount("azimuth", azimuth).

#include "tesser/node.h"
#include "tessermotion/axis.h"
#include "tessermotion/motion.h"

namespace tessermotion {

void describe(tesser::Object& o, Axis& axis);
void describe(tesser::Object& o, Motion& motion);

}  // namespace tessermotion
