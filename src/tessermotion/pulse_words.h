#pragma once

// Turns a PulseSource's periods into the words of a pulse train (docs/DESIGN.md
// section 4.9): each word is two durations with their levels, the form of an
// RMT symbol on ESP. A pulse is a word with the high part, then words of low
// part, as many as the period needs. Pure: the platform's refill copies the
// words into its memory, and the host tests read them.

#include <stddef.h>
#include <stdint.h>

#include "tessermotion/platform.h"

namespace tessermotion {

struct PulseWord {
    uint32_t duration0;  // ticks, 1..32767
    bool level0;
    uint32_t duration1;  // ticks, 1..32767; a word never has a zero duration
    bool level1;
};

class PulseWords {
public:
    // The longest duration a word can carry: 15 bits.
    static constexpr uint32_t kMaxDuration = 32767;

    // `highTicks`: the pulse's high part, 1..kMaxDuration.
    explicit PulseWords(uint32_t highTicks = 1) : highTicks_(highTicks) {}

    // Writes up to `max` words, taking periods from `source` as they are
    // needed. Sets `ended` when the source has no more periods; the words
    // written before that are still the train's. Returns the words written.
    size_t write(PulseSource& source, PulseWord* out, size_t max, bool& ended);

    // Starts a new train: a period half-written is dropped.
    void reset() {
        active_ = false;
        highPending_ = false;
        lowLeft_ = 0;
    }

private:
    uint32_t highTicks_;
    // The period being written: its high part (pending), then its low.
    bool active_ = false;
    bool highPending_ = false;
    uint32_t lowLeft_ = 0;
};

}  // namespace tessermotion
