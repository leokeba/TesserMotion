#pragma once

// TesserMotion: motion control and sensing on TesserAPI. See docs/DESIGN.md.

#include "tessermotion/accelerometer.h"
#include "tessermotion/adxl345.h"
#include "tessermotion/angle_correction.h"
#include "tessermotion/angle_sensor.h"
#include "tessermotion/axis.h"
#include "tessermotion/dc_motor.h"
#include "tessermotion/motion.h"
#include "tessermotion/mt6701.h"
#include "tessermotion/nmea.h"
#include "tessermotion/nmea_gps.h"
#include "tessermotion/platform.h"
#include "tessermotion/tb6612.h"
#include "tessermotion/tilt.h"
#include "tessermotion/trees.h"

#if defined(ESP_PLATFORM)
#include "tessermotion/platform/esp/i2c_bus.h"
#include "tessermotion/platform/esp/uart_port.h"
#endif
