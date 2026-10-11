#pragma once

// An I2C bus on ESP-IDF's i2c_master driver (docs/DESIGN.md section 7.1).

#if defined(ESP_PLATFORM)

#include "driver/i2c_master.h"
#include "tessermotion/platform.h"

namespace tessermotion {

struct I2cConfig {
    int sda;
    int scl;
    int port = -1;  // -1: the first free one
    uint32_t hz = 400000;
};

class I2cBus : public I2c {
public:
    // Transfers time out after this, so a stuck bus costs at most one tick.
    static constexpr int kTimeoutMs = 20;

    explicit I2cBus(const I2cConfig& config) : config_(config) {}
    // A bus the application already created (shared with other code); at
    // `hz` for the devices TesserMotion adds.
    explicit I2cBus(i2c_master_bus_handle_t bus, uint32_t hz = 400000) : bus_(bus) { config_.hz = hz; }

    bool begin() override;
    bool read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) override;
    bool write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) override;
    void recover() override;

private:
    i2c_master_dev_handle_t device(uint8_t addr);

    I2cConfig config_{-1, -1};
    i2c_master_bus_handle_t bus_ = nullptr;
    struct Device {
        uint8_t addr;
        i2c_master_dev_handle_t handle;
    };
    Device devices_[4] = {};
    size_t count_ = 0;
};

}  // namespace tessermotion

#endif
