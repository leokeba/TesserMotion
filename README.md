# TesserMotion

Motion control and sensing for ESP32 on [TesserAPI](https://github.com/leokeba/TesserAPI): DC motors, encoders and closed-loop axes, run by a fixed-rate control task. Every part describes itself into a TesserAPI tree, so it is reachable over HTTP, WebSocket and serial, and drawn by [TesserUI](https://github.com/leokeba/TesserUI) without UI code.

Status: early. A TB6612 motor driver, the MT6701 encoder, a closed-loop `Axis` (PID, limits, slew, sensor timeout) and the `Motion` control task, host-tested against a simulated motor; not yet run on hardware. Sequencers, an accelerometer and GPS come next. See [docs/DESIGN.md](docs/DESIGN.md).

```cpp
#include <TesserMotion.h>
using namespace tessermotion;

Tb6612Standby bridge(18);
Tb6612Channel motor(bridge, {.pwm = 16, .in1 = 17, .in2 = 5});
I2cBus bus({.sda = 13, .scl = 14});
Mt6701 encoder(bus);
Axis azimuth(encoder, motor);
Motion motion;

void setup() {
    motion.add(azimuth);
    api.mount("azimuth", azimuth);   // position, target, state, tuning, limits...
    motion.start(50);                // Hz
    azimuth.enable(true);            // axes boot disabled
}
```

Arduino-ESP32 ≥ 3.2 and ESP-IDF ≥ 5.2; ESP32, ESP32-S3, ESP32-C3. Examples: [examples/idf/axis](examples/idf/axis), [examples/arduino/Axis](examples/arduino/Axis), both on [TesserKIT](https://github.com/leokeba/TesserKIT) for Wi-Fi and the web interface.

## Host tests

```sh
cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
```

TesserAPI comes from `TESSERAPI_DIR` (default `../TesserAPI`), else GitHub. Applications can link `tessermotion_testing` for the fakes and the simulated plant in their own host tests.

MIT licensed.
