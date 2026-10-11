#if defined(ESP_PLATFORM)

#include "tessermotion/platform/esp/i2c_bus.h"

#include <string.h>

#include "esp_log.h"

namespace tessermotion {

namespace {
const char* kTag = "tessermotion";
}

bool I2cBus::begin() {
    if (bus_) return true;
    i2c_master_bus_config_t c = {};
    c.i2c_port = config_.port;
    c.sda_io_num = static_cast<gpio_num_t>(config_.sda);
    c.scl_io_num = static_cast<gpio_num_t>(config_.scl);
    c.clk_source = I2C_CLK_SRC_DEFAULT;
    c.glitch_ignore_cnt = 7;
    c.flags.enable_internal_pullup = true;
    esp_err_t err = i2c_new_master_bus(&c, &bus_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2C bus on SDA %d, SCL %d: %s", config_.sda, config_.scl, esp_err_to_name(err));
        bus_ = nullptr;
        return false;
    }
    return true;
}

i2c_master_dev_handle_t I2cBus::device(uint8_t addr) {
    for (size_t i = 0; i < count_; i++) {
        if (devices_[i].addr == addr) return devices_[i].handle;
    }
    if (!bus_ || count_ == sizeof devices_ / sizeof devices_[0]) return nullptr;
    i2c_device_config_t d = {};
    d.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    d.device_address = addr;
    d.scl_speed_hz = config_.hz;
    i2c_master_dev_handle_t h = nullptr;
    if (i2c_master_bus_add_device(bus_, &d, &h) != ESP_OK) return nullptr;
    devices_[count_++] = {addr, h};
    return h;
}

bool I2cBus::read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    i2c_master_dev_handle_t h = device(addr);
    return h && i2c_master_transmit_receive(h, &reg, 1, buf, len, kTimeoutMs) == ESP_OK;
}

bool I2cBus::write(uint8_t addr, uint8_t reg, const uint8_t* buf, size_t len) {
    uint8_t frame[17];
    i2c_master_dev_handle_t h = device(addr);
    if (!h || len + 1 > sizeof frame) return false;
    frame[0] = reg;
    memcpy(frame + 1, buf, len);
    return i2c_master_transmit(h, frame, len + 1, kTimeoutMs) == ESP_OK;
}

void I2cBus::recover() {
    // Clock pulses until the device holding SDA lets go, then a controller
    // reset.
    if (bus_) i2c_master_bus_reset(bus_);
}

}  // namespace tessermotion

#endif
