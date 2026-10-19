#if defined(ESP_PLATFORM)

#include "tessermotion/platform/esp/spi_bus.h"

#include "esp_log.h"

namespace tessermotion {

namespace {
const char* kTag = "tessermotion";
}

spi_host_device_t SpiBus::host() const {
    return config_.host < 0 ? SPI2_HOST : static_cast<spi_host_device_t>(config_.host);
}

bool SpiBus::begin() {
    if (begun_) return true;
    if (shared_) {
        begun_ = true;
        return true;
    }
    spi_bus_config_t c = {};
    c.sclk_io_num = config_.sclk;
    c.mosi_io_num = config_.mosi;
    c.miso_io_num = config_.miso;
    c.quadwp_io_num = -1;
    c.quadhd_io_num = -1;
    // DMA, so other code may share the bus with long transfers.
    esp_err_t err = spi_bus_initialize(host(), &c, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        // Initialized already, by other code: its pins stand.
        ESP_LOGW(kTag, "SPI host %d was already initialized: sharing it", static_cast<int>(host()));
    } else if (err != ESP_OK) {
        ESP_LOGE(kTag, "SPI bus on SCLK %d: %s", config_.sclk, esp_err_to_name(err));
        return false;
    }
    begun_ = true;
    return true;
}

spi_device_handle_t SpiBus::device(const SpiDevice& d) {
    for (size_t i = 0; i < count_; i++) {
        if (devices_[i].cs == d.cs) return devices_[i].handle;
    }
    if (!begun_ || count_ == kMaxDevices) return nullptr;
    spi_device_interface_config_t c = {};
    c.mode = d.mode;
    c.clock_speed_hz = static_cast<int>(d.hz);
    c.spics_io_num = d.cs;
    c.queue_size = 1;
    spi_device_handle_t h = nullptr;
    esp_err_t err = spi_bus_add_device(host(), &c, &h);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "SPI device on CS %d: %s", d.cs, esp_err_to_name(err));
        return nullptr;
    }
    devices_[count_++] = {d.cs, h};
    return h;
}

bool SpiBus::transfer(const SpiDevice& d, const uint8_t* tx, uint8_t* rx, size_t len) {
    spi_device_handle_t h = device(d);
    if (!h || len == 0) return false;
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    // Polling: a few bytes, done in microseconds; the driver arbitrates the
    // bus with other devices' transfers.
    return spi_device_polling_transmit(h, &t) == ESP_OK;
}

}  // namespace tessermotion

#endif
