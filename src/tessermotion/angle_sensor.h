#pragma once

// What an Axis reads (docs/DESIGN.md section 4.2).

namespace tessermotion {

class AngleSensor {
public:
    virtual ~AngleSensor() = default;
    virtual bool begin() = 0;
    // 0..360, raw; false on a bus error.
    virtual bool read(float& degrees) = 0;
};

}  // namespace tessermotion
