#pragma once

// TesserMotion: motion control and sensing on TesserAPI. See docs/DESIGN.md.

#include "tessermotion/angle_sensor.h"
#include "tessermotion/axis.h"
#include "tessermotion/dc_motor.h"
#include "tessermotion/motion.h"
#include "tessermotion/mt6701.h"
#include "tessermotion/platform.h"
#include "tessermotion/tb6612.h"
#include "tessermotion/trees.h"

#if defined(ESP_PLATFORM)
#include "tessermotion/platform/esp/i2c_bus.h"
#endif
