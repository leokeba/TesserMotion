#pragma once

// Motion's lock (docs/DESIGN.md section 3.5): a recursive FreeRTOS mutex on
// ESP, a std::recursive_mutex on the host. Recursive, so a part's update()
// (under the lock) may call another part's public methods (which take it).

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#else
#include <mutex>
#endif

namespace tessermotion {

class Lock {
public:
#if defined(ESP_PLATFORM)
    // Statically allocated, so safe to construct from global constructors.
    Lock() : handle_(xSemaphoreCreateRecursiveMutexStatic(&storage_)) {}
    void lock() { xSemaphoreTakeRecursive(handle_, portMAX_DELAY); }
    void unlock() { xSemaphoreGiveRecursive(handle_); }
#else
    Lock() = default;
    void lock() { mutex_.lock(); }
    void unlock() { mutex_.unlock(); }
#endif

    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;

private:
#if defined(ESP_PLATFORM)
    StaticSemaphore_t storage_;
    SemaphoreHandle_t handle_;
#else
    std::recursive_mutex mutex_;
#endif
};

// Holds `lock` for its scope; does nothing when `lock` is null (a part used
// without a Motion, from one task).
class Guard {
public:
    explicit Guard(Lock* lock) : lock_(lock) {
        if (lock_) lock_->lock();
    }
    ~Guard() {
        if (lock_) lock_->unlock();
    }
    Guard(const Guard&) = delete;
    Guard& operator=(const Guard&) = delete;

private:
    Lock* lock_;
};

}  // namespace tessermotion
