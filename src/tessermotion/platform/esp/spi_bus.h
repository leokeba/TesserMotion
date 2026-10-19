#pragma once

// An SPI bus on ESP-IDF's spi_master driver (docs/DESIGN.md section 7.4).

#if defined(ESP_PLATFORM)

#include "driver/spi_master.h"
#include "tessermotion/platform.h"

namespace tessermotion {

struct SpiConfig {
    int sclk;
    int mosi;
    int miso;
    int host = -1;  // -1: SPI2_HOST
};

class SpiBus : public Spi {
public:
    // Devices a bus holds: the hardware chip selects of a host (3 on the
    // ESP32's SPI2 and SPI3).
    static constexpr size_t kMaxDevices = 3;

    explicit SpiBus(const SpiConfig& config) : config_(config) {}
    // A bus other code already initialized (an Ethernet chip's, say):
    // TesserMotion only adds its devices to it.
    explicit SpiBus(spi_host_device_t host) : shared_(true) { config_.host = static_cast<int>(host); }

    bool begin() override;
    bool transfer(const SpiDevice& device, const uint8_t* tx, uint8_t* rx, size_t len) override;

private:
    spi_host_device_t host() const;
    spi_device_handle_t device(const SpiDevice& d);

    SpiConfig config_{-1, -1, -1};
    bool shared_ = false;
    bool begun_ = false;
    struct Device {
        int cs;
        spi_device_handle_t handle;
    };
    Device devices_[kMaxDevices] = {};
    size_t count_ = 0;
};

}  // namespace tessermotion

#endif
