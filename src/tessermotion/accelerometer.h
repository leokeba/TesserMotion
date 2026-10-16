#pragma once

// What a Tilt reads (docs/DESIGN.md section 6.1).

namespace tessermotion {

class Accelerometer {
public:
    virtual ~Accelerometer() = default;
    // False when the sensor doesn't answer as itself.
    virtual bool begin() = 0;
    // m/s²; false on a bus error.
    virtual bool read(float& x, float& y, float& z) = 0;
};

}  // namespace tessermotion
