#if defined(ESP_PLATFORM)

#include "tessermotion/platform/esp/uart_port.h"

#include "driver/uart.h"
#include "esp_log.h"

namespace tessermotion {

namespace {
const char* kTag = "tessermotion";
}

bool UartPort::begin() {
    if (installed_) return true;
    uart_port_t port = static_cast<uart_port_t>(config_.port);
    if (uart_is_driver_installed(port)) {
        // Someone else reads it (Arduino's Serial1, for one).
        ESP_LOGE(kTag, "UART %d is already in use", config_.port);
        return false;
    }
    uart_config_t c = {};
    c.baud_rate = static_cast<int>(config_.baud);
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;
    esp_err_t err = uart_param_config(port, &c);
    if (err == ESP_OK) {
        err = uart_set_pin(port, config_.tx >= 0 ? config_.tx : UART_PIN_NO_CHANGE, config_.rx, UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) err = uart_driver_install(port, kRxBufferBytes, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "UART %d on RX %d: %s", config_.port, config_.rx, esp_err_to_name(err));
        return false;
    }
    installed_ = true;
    return true;
}

size_t UartPort::read(uint8_t* buf, size_t max) {
    if (!installed_ || max == 0) return 0;
    int n = uart_read_bytes(static_cast<uart_port_t>(config_.port), buf, static_cast<uint32_t>(max), 0);
    return n > 0 ? static_cast<size_t>(n) : 0;
}

size_t UartPort::read(uint8_t* buf, size_t len, uint32_t timeoutMs) {
    if (!installed_ || len == 0) return 0;
    // Rounded up, plus a tick: a wait of n ticks ends at the n-th tick
    // interrupt, which may come at once. pdMS_TO_TICKS() rounds down, so at
    // 100 Hz a 10 ms wait was 1 tick, anywhere from 0 to 10 ms, and a 1.1 ms
    // TMC2209 reply straddling a tick timed out.
    TickType_t wait = static_cast<TickType_t>((uint64_t(timeoutMs) * configTICK_RATE_HZ + 999) / 1000 + 1);
    int n = uart_read_bytes(static_cast<uart_port_t>(config_.port), buf, static_cast<uint32_t>(len), wait);
    return n > 0 ? static_cast<size_t>(n) : 0;
}

bool UartPort::write(const uint8_t* buf, size_t len) {
    if (!installed_ || config_.tx < 0) return false;
    return uart_write_bytes(static_cast<uart_port_t>(config_.port), buf, len) == static_cast<int>(len);
}

}  // namespace tessermotion

#endif
