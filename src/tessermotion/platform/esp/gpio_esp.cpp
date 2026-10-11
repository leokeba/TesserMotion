#if defined(ESP_PLATFORM)

#include "driver/gpio.h"
#include "tessermotion/platform.h"

namespace tessermotion {

namespace {

class EspGpio : public Gpio {
public:
    void output(int pin, bool level) override {
        gpio_num_t p = static_cast<gpio_num_t>(pin);
        gpio_reset_pin(p);
        gpio_set_level(p, level);
        gpio_set_direction(p, GPIO_MODE_OUTPUT);
    }
    void set(int pin, bool level) override { gpio_set_level(static_cast<gpio_num_t>(pin), level); }
};

}  // namespace

Gpio& defaultGpio() {
    static EspGpio gpio;
    return gpio;
}

}  // namespace tessermotion

#endif
