#if defined(ESP_PLATFORM)

#include "tessermotion/platform/esp/pulse_rmt.h"

#include "esp_attr.h"
#include "esp_log.h"

namespace tessermotion {

namespace {

const char* kTag = "tessermotion";
constexpr size_t kMemoryBlockSymbols = 64;
constexpr size_t kMinChunk = 8;
constexpr size_t kMaxWords = 16;  // per refill: a sixteenth of a block at most

}  // namespace

bool RmtPulseOutput::begin(int pin, uint32_t tickHz, uint32_t highTicks) {
    if (channel_) return true;
    rmt_tx_channel_config_t config = {};
    config.gpio_num = static_cast<gpio_num_t>(pin);
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = tickHz;
    config.mem_block_symbols = kMemoryBlockSymbols;
    config.trans_queue_depth = 1;
    esp_err_t err = rmt_new_tx_channel(&config, &channel_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "RMT TX channel on GPIO %d: %s", pin, esp_err_to_name(err));
        return false;
    }

    const auto discard = [this, pin]() {
        if (encoder_) {
            const esp_err_t deleteErr = rmt_del_encoder(encoder_);
            if (deleteErr != ESP_OK) ESP_LOGE(kTag, "RMT encoder cleanup on GPIO %d: %s", pin, esp_err_to_name(deleteErr));
            encoder_ = nullptr;
        }
        if (channel_) {
            const esp_err_t deleteErr = rmt_del_channel(channel_);
            if (deleteErr != ESP_OK) ESP_LOGE(kTag, "RMT channel cleanup on GPIO %d: %s", pin, esp_err_to_name(deleteErr));
            channel_ = nullptr;
        }
    };

    rmt_tx_event_callbacks_t callbacks = {};
    callbacks.on_trans_done = onDone;
    err = rmt_tx_register_event_callbacks(channel_, &callbacks, this);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "RMT TX callback registration on GPIO %d: %s", pin, esp_err_to_name(err));
        discard();
        return false;
    }

    rmt_simple_encoder_config_t encoder = {};
    encoder.callback = encodeCb;
    encoder.arg = this;
    encoder.min_chunk_size = kMinChunk;
    err = rmt_new_simple_encoder(&encoder, &encoder_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "RMT simple encoder on GPIO %d: %s", pin, esp_err_to_name(err));
        discard();
        return false;
    }
    err = rmt_enable(channel_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "RMT TX enable on GPIO %d: %s", pin, esp_err_to_name(err));
        discard();
        return false;
    }
    words_ = PulseWords(highTicks > 0 && highTicks <= PulseWords::kMaxDuration ? highTicks : 1);
    return true;
}

bool RmtPulseOutput::start(PulseSource& source) {
    if (!channel_ || !encoder_) {
        ESP_LOGE(kTag, "RMT TX start requested before GPIO channel initialization");
        return false;
    }
    if (running_.load()) return false;
    source_ = &source;
    words_.reset();
    stopping_.store(false);
    running_.store(true);
    rmt_transmit_config_t transmit = {};
    transmit.loop_count = 0;
    const esp_err_t err = rmt_transmit(channel_, encoder_, &payload_, sizeof payload_, &transmit);
    if (err != ESP_OK) {
        running_.store(false);
        source_ = nullptr;
        ESP_LOGE(kTag, "RMT TX start: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

void RmtPulseOutput::stop() {
    if (!channel_) return;
    stopping_.store(true);
    const esp_err_t err = rmt_tx_wait_all_done(channel_, 100);
    if (err != ESP_OK) ESP_LOGE(kTag, "RMT TX stop: %s", esp_err_to_name(err));
    running_.store(false);
    source_ = nullptr;
}

bool IRAM_ATTR RmtPulseOutput::onDone(rmt_channel_handle_t, const rmt_tx_done_event_data_t*, void* arg) {
    static_cast<RmtPulseOutput*>(arg)->running_.store(false);
    return false;
}

size_t IRAM_ATTR RmtPulseOutput::encodeCb(const void*, size_t, size_t, size_t free, rmt_symbol_word_t* symbols,
                                          bool* done, void* arg) {
    return static_cast<RmtPulseOutput*>(arg)->encode(symbols, free, done);
}

size_t IRAM_ATTR RmtPulseOutput::encode(rmt_symbol_word_t* symbols, size_t free, bool* done) {
    if (stopping_.load() || !source_) {
        *done = true;
        return 0;
    }
    PulseWord words[kMaxWords];
    const size_t max = free < kMaxWords ? free : kMaxWords;
    bool ended = false;
    const size_t n = words_.write(*source_, words, max, ended);
    for (size_t i = 0; i < n; i++) {
        // The symbol's layout: duration0 (15 bits), level0, duration1, level1.
        symbols[i].val = (words[i].duration0 & 0x7FFFu) | (words[i].level0 ? 0x8000u : 0u) |
                         ((words[i].duration1 & 0x7FFFu) << 16) | (words[i].level1 ? 0x80000000u : 0u);
    }
    if (ended) *done = true;
    return n;
}

}  // namespace tessermotion

#endif
