#include "tessermotion/tb6612.h"

namespace tessermotion {

void Tb6612Standby::begin() {
    if (begun_) return;
    if (!gpio_) gpio_ = &defaultGpio();
    gpio_->output(pin_, true);
    begun_ = true;
}

void Tb6612Standby::enable(bool on) {
    if (begun_) gpio_->set(pin_, on);
}

bool Tb6612Channel::begin() {
    if (!gpio_) gpio_ = &defaultGpio();
    if (!pwm_) pwm_ = &defaultPwm();
    gpio_->output(pins_.in1, true);
    gpio_->output(pins_.in2, false);
    channel_ = pwm_->attach(pins_.pwm, pins_.frequencyHz);
    if (channel_ < 0) return false;
    standby_.begin();
    return true;
}

void Tb6612Channel::drive(bool forward, float duty) {
    if (channel_ < 0) return;
    gpio_->set(pins_.in1, forward);
    gpio_->set(pins_.in2, !forward);
    pwm_->duty(channel_, duty);
}

}  // namespace tessermotion
