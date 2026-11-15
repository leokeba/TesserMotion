#include "tessermotion/pulse_words.h"

namespace tessermotion {

size_t PulseWords::write(PulseSource& source, PulseWord* out, size_t max, bool& ended) {
    size_t n = 0;
    while (n < max) {
        if (!active_) {
            uint32_t period = 0;
            if (source.nextPeriods(&period, 1) == 0) {
                ended = true;
                break;
            }
            // A period is at least the pulse's high part plus a tick.
            lowLeft_ = period > highTicks_ ? period - highTicks_ : 1;
            active_ = true;
            highPending_ = true;
        }
        PulseWord word = {};
        if (highPending_) {
            // The pulse, then the first part of its low: one word.
            uint32_t low = lowLeft_ > kMaxDuration ? kMaxDuration : lowLeft_;
            if (low == 0) low = 1;  // never a zero duration: it ends the transmission
            lowLeft_ = low < lowLeft_ ? lowLeft_ - low : 0;
            word.duration0 = highTicks_;
            word.level0 = true;
            word.duration1 = low;
            word.level1 = false;
            highPending_ = false;
        } else {
            // The rest of the low, two durations a word. A last tick takes a
            // second word's worth of its own (a tenth of a microsecond at 10 MHz).
            uint32_t a = lowLeft_ > kMaxDuration ? kMaxDuration : lowLeft_;
            lowLeft_ -= a;
            uint32_t b = lowLeft_ > kMaxDuration ? kMaxDuration : lowLeft_;
            lowLeft_ -= b;
            if (b == 0) {
                if (a >= 2) {
                    b = a / 2;
                    a -= b;
                } else {
                    b = 1;
                }
            }
            word.duration0 = a;
            word.level0 = false;
            word.duration1 = b;
            word.level1 = false;
        }
        out[n++] = word;
        if (!highPending_ && lowLeft_ == 0) active_ = false;
    }
    return n;
}

}  // namespace tessermotion
