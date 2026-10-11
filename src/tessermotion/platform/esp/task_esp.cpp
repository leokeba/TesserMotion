#if defined(ESP_PLATFORM)

#include <utility>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tessermotion/platform.h"

namespace tessermotion {

namespace {

struct Periodic {
    TickType_t period;
    std::function<void(uint64_t)> tick;
};

void run(void* arg) {
    auto* p = static_cast<Periodic*>(arg);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        p->tick(nowUs());
        vTaskDelayUntil(&last, p->period);
    }
}

}  // namespace

uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }

bool startPeriodic(const TaskConfig& config, uint32_t periodUs, std::function<void(uint64_t)> tick) {
    if (!tick) return false;
    TickType_t period = pdMS_TO_TICKS(periodUs / 1000);
    auto* p = new Periodic{period > 0 ? period : 1, std::move(tick)};
    UBaseType_t priority = config.priority < 0 ? configMAX_PRIORITIES - 2 : static_cast<UBaseType_t>(config.priority);
    BaseType_t core = config.core >= 0 ? config.core : (portNUM_PROCESSORS > 1 ? 1 : tskNO_AFFINITY);
    if (xTaskCreatePinnedToCore(run, config.name, config.stackBytes, p, priority, nullptr, core) != pdPASS) {
        delete p;
        return false;
    }
    return true;
}

}  // namespace tessermotion

#endif
