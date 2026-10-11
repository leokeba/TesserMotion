#if defined(ESP_PLATFORM)

#include "driver/ledc.h"
#include "esp_log.h"
#include "soc/soc_caps.h"
#include "tessermotion/platform.h"

namespace tessermotion {

namespace {

const char* kTag = "tessermotion";
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr uint32_t kClockHz = 80000000;  // APB, LEDC_AUTO_CLK's choice at these frequencies

// LEDC (docs/DESIGN.md section 7.2). Timers and channels are taken from the
// top down, as Arduino's analogWrite() and ledcAttach() take them from the
// bottom up. Outputs at the same frequency share a timer.
class EspPwm : public Pwm {
public:
    int attach(int pin, uint32_t hz) override {
        int timer = timerFor(hz);
        if (timer < 0 || nextChannel_ < 0) {
            ESP_LOGE(kTag, "PWM on GPIO %d: no LEDC %s left", pin, timer < 0 ? "timer" : "channel");
            return -1;
        }
        ledc_channel_config_t c = {};
        c.gpio_num = pin;
        c.speed_mode = kMode;
        c.channel = static_cast<ledc_channel_t>(nextChannel_);
        c.timer_sel = static_cast<ledc_timer_t>(timer);
        c.duty = 0;
        if (ledc_channel_config(&c) != ESP_OK) return -1;
        bits_[nextChannel_] = timers_[timer].bits;
        return nextChannel_--;
    }

    void duty(int channel, float fraction) override {
        if (channel < 0 || channel >= SOC_LEDC_CHANNEL_NUM) return;
        fraction = fraction < 0 ? 0 : fraction > 1 ? 1 : fraction;
        uint32_t full = 1u << bits_[channel];
        auto d = static_cast<uint32_t>(fraction * static_cast<float>(full) + 0.5f);
        ledc_set_duty(kMode, static_cast<ledc_channel_t>(channel), d);
        ledc_update_duty(kMode, static_cast<ledc_channel_t>(channel));
    }

private:
    struct Timer {
        uint32_t hz;
        uint8_t bits;
    };

    int timerFor(uint32_t hz) {
        for (int t = LEDC_TIMER_MAX - 1; t > nextTimer_; t--) {
            if (timers_[t].hz == hz) return t;
        }
        if (nextTimer_ < 0 || hz == 0) return -1;
        // The finest resolution the frequency allows: 12 bits at 15 kHz.
        uint8_t bits = 1;
        while (bits < SOC_LEDC_TIMER_BIT_WIDTH && (kClockHz >> (bits + 1)) >= hz) bits++;
        ledc_timer_config_t c = {};
        c.speed_mode = kMode;
        c.duty_resolution = static_cast<ledc_timer_bit_t>(bits);
        c.timer_num = static_cast<ledc_timer_t>(nextTimer_);
        c.freq_hz = hz;
        c.clk_cfg = LEDC_AUTO_CLK;
        if (ledc_timer_config(&c) != ESP_OK) return -1;
        timers_[nextTimer_] = {hz, bits};
        return nextTimer_--;
    }

    Timer timers_[LEDC_TIMER_MAX] = {};
    int nextTimer_ = LEDC_TIMER_MAX - 1;
    uint8_t bits_[SOC_LEDC_CHANNEL_NUM] = {};
    int nextChannel_ = SOC_LEDC_CHANNEL_NUM - 1;
};

}  // namespace

Pwm& defaultPwm() {
    static EspPwm pwm;
    return pwm;
}

}  // namespace tessermotion

#endif
