#pragma once

// A step output on ESP-IDF's RMT (docs/DESIGN.md section 4.9): one channel per
// output, its pulses encoded on demand by a simple callback encoder, so a
// ramp's periods come from the source as the channel needs them. Only the
// refill runs per memory block, not per step.

#if defined(ESP_PLATFORM)

#include <stdint.h>

#include <atomic>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "tessermotion/platform.h"
#include "tessermotion/pulse_words.h"

namespace tessermotion {

class RmtPulseOutput : public PulseOutput {
public:
    // The first begin() sets the channel up; later calls return true without
    // changing it.
    bool begin(int pin, uint32_t tickHz, uint32_t highTicks) override;
    bool start(PulseSource& source) override;
    bool running() override { return running_.load(std::memory_order_acquire); }
    // Lets the pulses already in the channel's memory play (at most a block),
    // with the source not called again, then returns.
    void stop() override;

private:
    static bool onDone(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t* event, void* arg);
    static size_t encodeCb(const void* data, size_t dataSize, size_t written, size_t free, rmt_symbol_word_t* symbols,
                           bool* done, void* arg);
    size_t encode(rmt_symbol_word_t* symbols, size_t free, bool* done);

    rmt_channel_handle_t channel_ = nullptr;
    rmt_encoder_handle_t encoder_ = nullptr;
    PulseSource* source_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopping_{false};
    uint8_t payload_ = 0;  // the encoder's data: none; the transaction needs one byte

    // The periods as words, written a block at a time.
    PulseWords words_;
};

}  // namespace tessermotion

#endif
