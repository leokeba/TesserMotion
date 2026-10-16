#pragma once

// A UART on ESP-IDF's driver, receive only (docs/DESIGN.md section 7.3).

#if defined(ESP_PLATFORM)

#include "tessermotion/platform.h"

namespace tessermotion {

struct UartConfig {
    int rx;
    int tx = -1;  // -1: not connected
    uint32_t baud = 9600;
    int port = 1;
};

class UartPort : public Uart {
public:
    // The driver's receive buffer. The control task drains it every tick,
    // which is 20 bytes at 9600 baud and 50 Hz.
    static constexpr int kRxBufferBytes = 256;

    explicit UartPort(const UartConfig& config) : config_(config) {}

    bool begin() override;
    size_t read(uint8_t* buf, size_t max) override;

private:
    UartConfig config_;
    bool installed_ = false;
};

}  // namespace tessermotion

#endif
