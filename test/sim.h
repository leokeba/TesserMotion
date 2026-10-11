// One axis on a simulated plant, ticked by a Motion with a fake clock.
#pragma once

#include <math.h>

#include <functional>

#include "TesserMotion.h"
#include "check.h"
#include "fake_hw.h"
#include "plant.h"
#include "tessermotion/angle.h"

struct Sim {
    Plant plant;
    tessermotion::Axis axis{plant, plant};
    tessermotion::Motion motion;
    uint64_t now = 1000000;
    uint32_t periodUs;

    explicit Sim(float rateHz = 50, float start = 0, PlantParams params = PlantParams(), uint32_t seed = 1)
        : plant(start, params, seed), periodUs(static_cast<uint32_t>(1e6f / rateHz)) {
        motion.add(axis);
        motion.begin();
    }

    void tick() {
        plant.advance(static_cast<float>(periodUs) * 1e-6f);
        now += periodUs;
        motion.step(now);
    }
    // `each` runs after every tick.
    void run(float seconds, const std::function<void()>& each = nullptr) {
        auto n = static_cast<int>(seconds * 1e6f / static_cast<float>(periodUs) + 0.5f);
        for (int i = 0; i < n; i++) {
            tick();
            if (each) each();
        }
    }
    // Enabled and holding where it is.
    void start() {
        axis.enable(true);
        run(0.1f);
    }
};

// The shortest distance between two angles.
inline float apart(float a, float b) { return fabsf(tessermotion::wrap180(a - b)); }
