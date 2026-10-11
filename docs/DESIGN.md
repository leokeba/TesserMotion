# TesserMotion design

Status: draft, written before any code (2026-10-11). Everything here is the specification the code will follow; when the two disagree, fix one of them. Sections marked *(later)* are committed to but not part of the first release. References to TesserAPI, TesserUI and TesserKIT point to their own DESIGN.md: [TesserAPI](https://github.com/leokeba/TesserAPI/blob/main/docs/DESIGN.md), [TesserUI](https://github.com/leokeba/TesserUI/blob/main/docs/DESIGN.md), [TesserKIT](https://github.com/leokeba/TesserKIT/blob/main/docs/DESIGN.md).

## 1. Goals

TesserMotion is a library of motion control and sensing for ESP32 devices built on [TesserAPI](https://github.com/leokeba/TesserAPI): motor drivers, encoders, closed-loop axes, keyframe sequencers, and the sensors that go with them (accelerometer, GPS). Every part describes itself into a TesserAPI tree, so it is reachable over every transport and drawn by [TesserUI](https://github.com/leokeba/TesserUI) without UI code. Its first user is the heliostat (`~/Dev/open-heliostat`, branch `helionext`), ported to [TesserKIT](https://github.com/leokeba/TesserKIT) in TesserKIT's phase 3. Its second will be the spin sequencer (`~/Dev/spin-sequencer`), once steppers are added (§13).

- **A library, separate from TesserKIT.** TesserKIT holds what every connected device needs. Motion is what some devices do. A TesserKIT device without motors never compiles or downloads this code, and TesserMotion doesn't depend on TesserKIT (§3.4).
- **Pay for what you use.** A part that isn't instantiated isn't linked: no static registration, no feature flags. Chip-specific drivers compile only on chips that have the peripheral.
- **Three layers per part.** A driver talks to hardware. A controller holds the logic and is host-tested. A tree adds the TesserAPI description. Each layer is usable without the ones above it (§3.1).
- **Zero UI code.** Parts carry TesserUI hints. The keyframe editor is TesserUI's `x-keyframes` example element (TesserUI §9.8), whose tree shape the sequencer follows exactly.
- **Host-tested against a simulated plant.** The PID, limits, slew, autotune and sequencer run on the host against a model of a geared DC motor with a quantized, noisy encoder (§11).
- **Same behaviour as the heliostat, minus its bugs.** The port keeps every control feature the heliostat has today (§4.3), and fixes the defects listed in §4.6.

### Non-goals

- Application geometry: sun position, mount orientation solving, the joint accelerometer and encoder calibration, sources and targets. They stay in the heliostat.
- Steppers in the first release (§13).
- Motion planning beyond one axis: synchronized multi-axis moves come from a sequencer with one track per axis (§5), not from a planner.

## 2. Constraints

| Item | Decision |
|---|---|
| Language | C++17, TesserAPI's rules: no exceptions, no RTTI, never `throw`/`dynamic_cast`/`typeid`; calling an empty `std::function` aborts, so check it first. Virtual functions are fine. |
| Frameworks | ESP-IDF ≥ 5.2 and Arduino-ESP32 ≥ 3.1 (the I2C master driver, §7.1). The same code on both: ESP-IDF drivers only, no Arduino APIs (no `Wire`, `ledcWrite`, `HardwareSerial`, `millis`). |
| Chips | Classic ESP32 (reference: the heliostat), ESP32-S3, ESP32-C3. Drivers use LEDC, GPIO, I2C and UART, which every target has. MCPWM and PCNT drivers, if added, are guarded by `SOC_MCPWM_SUPPORTED` / `SOC_PCNT_SUPPORTED` (the C3 has neither). |
| Numbers | `float` in all control code: the ESP32 and the S3 have a single-precision FPU and do `double` in software. Times are integers: `uint64_t` microseconds from the platform clock, `uint32_t` milliseconds in keyframes. |
| Dependencies | TesserAPI (and ArduinoJson 7 through it). No third-party driver libraries: the heliostat's TinyGPSPlus, TimeLib and teleplot are replaced (§6.2, §9.1). |
| Platform code | Only in `src/tessermotion/platform/`. Everything else sees the narrow interfaces of §3.3 and is host-tested against fakes. |
| Threading | One control task (§3.2). Hardware is touched only from it, except at `begin()`. |

## 3. Architecture

```
 application ──▶ Motion (control task, fixed rate)
                   │  tick: sensors → sequencers → axes
                   ├─▶ Sequencer ──(tracks)──▶ Axis target, or a function
                   ├─▶ Axis ──▶ AngleSensor (Mt6701 on I2cBus)
                   │        └─▶ DcMotor (Tb6612Channel on Pwm + Gpio)
                   └─▶ Accelerometer (Adxl345 on I2cBus), Gps (Nmea on Uart)

 tesser::Api ◀── describe(Object&, Axis&), describe(Object&, Sequencer&), ...
```

### 3.1 Layers

| Layer | Content | Depends on |
|---|---|---|
| Driver | `Tb6612Channel`, `Tb6612Standby`, `Mt6701`, `Adxl345`, `NmeaGps`: register maps, pin logic, scaling. | The platform interfaces (§3.3). |
| Controller | `DcMotor` output mapping, `Pid`, `RelayAutotune`, `Axis`, `Sequencer`, `Motion`. Pure logic; time comes in as an argument. | Drivers through their interfaces (`AngleSensor`, `Motor`, `Accelerometer`). |
| Tree | `describe()` free functions, found by argument-dependent lookup as TesserAPI's `mount()` expects (TesserAPI §4.3), in their own `*_tree.cpp` files. | TesserAPI. |

Headers of the first two layers don't include TesserAPI, so a project can use an `Axis` without a tree. The package still lists TesserAPI as a dependency.

### 3.2 The control task

```cpp
#include <TesserMotion.h>
using namespace tessermotion;

Tb6612Standby driver(GPIO_NUM_18);
Tb6612Channel azMotor(driver, {.pwm = 16, .in1 = 17, .in2 = 5});
I2cBus bus1({.sda = 13, .scl = 14}), bus2({.sda = 27, .scl = 26});
Mt6701 azSensor(bus1);
Axis azimuth(azSensor, azMotor);
Sequencer azSequence(1);           // one track
Adxl345 tilt(bus1, 0x53);
Motion motion;

void setup() {
    azSequence.bind(0, azimuth);
    motion.add(tilt);
    motion.add(azSequence);
    motion.add(azimuth);
    api.mount("azimuth", azimuth);
    api.object("azimuth").mount("sequence", azSequence);
    api.mount("tilt", tilt);
    motion.begin({.rateHz = 50, .priority = configMAX_PRIORITIES - 2, .core = 1});
}
```

- **One task, fixed rate.** `Motion::begin()` starts a task that calls `step(nowUs)` every period with `vTaskDelayUntil`. Defaults: 50 Hz, priority `configMAX_PRIORITIES - 2`, core 1 on dual-core chips, 4 KB stack. The heliostat runs at 20 Hz today; 50 Hz is the new default, to be confirmed on hardware (§11).
- **Tick order:** sensors read (outside the lock, §3.5), then sequencers write targets, then axes compute and apply outputs, then `onTick` callbacks.
- **`step(nowUs)` is public.** Tests call it directly with a fake clock, without a task. So does an application that wants to drive the loop itself (`begin()` not called).
- **Constructors do nothing but store arguments** (the static-initialization rule, TesserKIT's AGENTS.md). Hardware setup happens in `Motion::begin()`, which calls each part's `begin()` in the order they were added.
- **Statistics:** the task measures its period and the duration of each tick. `Motion` reports `period` (ms, the last), `tickMax` (µs since the last read) and `overruns` (ticks that took longer than the period) as read-only values (§9.4). These replace the heliostat's "Servo loop lag" and "Control loop slow" prints.
- **`stopAll()`** disables every axis and stops every sequencer, from any task. An application calls it before a firmware update (TesserKIT's `dev.onState()`, state `Updating`) and from its own emergency paths.

### 3.3 Platform layer

Narrow interfaces, an ESP implementation in `src/tessermotion/platform/esp/`, and fakes in `test/fake_hw.*`. As in TesserKIT, a part takes the interfaces it needs, so an interface without a user isn't linked.

| Interface | Methods | ESP implementation |
|---|---|---|
| `Clock` | `nowUs()` | `esp_timer_get_time()` |
| `Gpio` | `output(pin, level)`, `set(pin, level)` | `driver/gpio.h` |
| `Pwm` | `attach(pin, frequencyHz) → channel`, `duty(channel, fraction 0..1)` | LEDC (§7.2) |
| `I2cBus` | `read(addr, reg, buf, len)`, `write(addr, reg, buf, len)`, `recover()` (nine SCL pulses, re-initialize), each with a timeout | `driver/i2c_master.h` (§7.1) |
| `Uart` | `begin(rx, tx, baud)`, non-blocking `read(buf, max) → n` | `driver/uart.h` |
| `Lock` | `lock()`, `unlock()` | a FreeRTOS mutex; `std::mutex` on the host |
| `Task` | `start(name, stack, priority, core, fn)` | `xTaskCreatePinnedToCore` |

`I2cBus` and `Uart` are concrete classes on ESP (`I2cBus bus({.sda, .scl, .port, .hz})`) that implement the interface. A bus can also wrap a handle the application already created (`I2cBus bus(existingHandle)`), so TesserMotion shares a bus with other code.

### 3.4 What each layer owns

| Concern | Owner |
|---|---|
| The tree, schema, validation, persistence, locking of the tree | TesserAPI |
| Rendering, widgets, charts, the `x-keyframes` element | TesserUI |
| Wi-Fi, time, firmware updates, backups, the device's task | TesserKIT, when the application uses it |
| Drivers, control, sequencing, sensors, their trees | TesserMotion |
| Geometry, what to point at, sources and targets | the application |

A gap in TesserAPI or TesserUI goes to that repository, generically, with its tests and spec update (TesserKIT DESIGN §3.4), not to a workaround here. The example firmware may use TesserKIT for Wi-Fi and the web interface. The library never includes it.

### 3.5 Concurrency

Two kinds of tasks touch a part: transports change settings and read status through the tree, under the API lock (TesserAPI §9), and the control task runs the loop.

- **`Motion` has one lock of its own** (`Lock`, §3.3). The control task holds it while sequencers and axes compute and apply outputs, which is tens of microseconds, never during bus transfers. Tree getters and setters take it around the fields they touch.
- **Sensor reads happen before the lock.** A sensor's result is stored in the part, then picked up under the lock. A slow or failing I2C transfer (up to its timeout) therefore never blocks a tree request.
- **Lock order:** the API lock, then Motion's lock. The control task never takes the API lock. Status values reach subscribers because TesserAPI samples watched getters under its own lock, and those getters take Motion's lock briefly.
- **Only the control task drives hardware.** A tree write records a request (a new target, enable, autotune start), and the next tick applies it. No setter calls a motor or a sensor. This removes the heliostat's race, where `setAngle()` called `run()` from the HTTP task while the control task was running (§4.6).

## 4. DC motors and axes

### 4.1 Motor interface and the TB6612

```cpp
class Motor {                       // what an Axis drives
public:
    virtual void begin() = 0;
    virtual void output(float u) = 0;   // -1..1, sign is direction
    virtual void brake() = 0;           // short brake
    virtual void coast() = 0;           // outputs off, high impedance
};
```

`DcMotor` implements the output mapping and holds the settings. `Tb6612Channel` is a `DcMotor` on one channel of a TB6612FNG (PWM, IN1, IN2), sharing a `Tb6612Standby` (the STBY pin) with the other channel.

- **Output mapping** (the heliostat's, kept): for |u| ≥ 0.001, duty = |u|^`shape` × (1 − `minOutput`) + `minOutput`; below that, 0. `minOutput` compensates the motor's dead band, and `shape` curves the response. `invert` swaps the direction.
- **TB6612 truth table:** IN1 H / IN2 L forward, L / H reverse, H / H short brake, L / L coast. PWM low gives a short brake between pulses (the chip's slow decay). `output(0)` keeps the direction pins and sets duty 0, which brakes: the heliostat's behaviour, which holds a mirror against wind. `coast()` is L / L.
- **Standby:** `Tb6612Standby::begin()` drives STBY high once every channel on it has started. `stopAll()` and a sensor fault don't touch STBY; `Tb6612Standby::enable(false)` is for the application.
- **PWM:** 15 kHz by default (the heliostat's), configurable from 1 to 40 kHz. The LEDC resolution is the highest the frequency allows on the chip (12 bits at 15 kHz on the ESP32).

### 4.2 Angle sensors

```cpp
class AngleSensor {
public:
    virtual bool begin() = 0;
    virtual bool read(float& degrees) = 0;   // 0..360, raw; false on a bus error
};
```

`Mt6701` reads an MT6701 magnetic encoder over I2C (address 0x06): a 14-bit angle in registers 0x03 (bits 13–6) and 0x04 (bits 7–2 hold bits 5–0). The heliostat reads three bytes from 0x02 and uses the last two. Read 0x03–0x04 and check on hardware that it gives the same values, keeping the three-byte read if it doesn't. Each MT6701 has the same fixed address, so two encoders need two buses (the heliostat uses I2C 0 and 1).

**Bus faults** (the heliostat's recovery, kept): after 3 failed reads in a row, or one transfer longer than its timeout (20 ms), the sensor calls `bus.recover()`, at most every 100 ms. The axis sees `read()` return false and counts it (§4.3, §9.1).

### 4.3 Axis

An `Axis` is a closed-loop position servo: an `AngleSensor`, a `Motor`, and a PID between them, in degrees on a circle. It keeps every feature of the heliostat's `Servo_Driver`:

- **Sensor frame:** position = (raw, inverted if `sensor.invert`) + `sensor.offset`, modulo 360.
- **Limits:** an arc from `begin` to `end`, clockwise (increasing angles, modulo 360). A target outside the arc is clamped to the nearer end, measured from the arc's middle. With limits on, the error is computed in a frame centred on the arc's middle, so the axis never moves through the forbidden zone. Without limits, the error is the shortest way around.
- **Limits are physical.** They are stored internally in the raw sensor frame and shown in the output frame. Changing `offset` therefore leaves them at the same mechanical stops, as in the heliostat, and loading persisted settings gives the same result in any order. The heliostat shifts the limits in the offset's setter, which depends on load order.
- **PID**, on error e (degrees) with the measured period dt:
  - integral += e·dt, clamped to ±`integralLimit` / |ki| (so the integral term alone never exceeds `integralLimit` of full output);
  - derivative = low-pass(`derivativeFilter`) of ½·de/dt − ½·dposition/dt, each clamped to ±`maxDerivative` (°/s), which mixes derivative on error and on measurement;
  - u = kp·e + ki·integral + kd·derivative, rate-limited by `ramp` (full scale per second, 0 = off), clamped to ±1;
  - inside `tolerance`, u = 0 and the integral and derivative reset.
- **Slew:** with `maxSpeed` > 0 (°/s), a new target is reached by a linear ramp of the setpoint along the error's path. Changing `maxSpeed` during a ramp recomputes the rest from the current setpoint. With 0, the setpoint jumps.
- **Sensor timeout:** without a good reading for `sensorTimeout` (200 ms), the axis outputs 0 and its state is `sensorFault`. It resumes when readings come back, towards the same target.
- **Start:** an enabled axis waits for its first good reading, sets its target to the position (so it never jumps at boot), then runs.
- **Disabled:** output 0 (brake), integral reset. The motor can then be driven by hand through `motor/output` (§9.1).

**States** (an enum, read-only, watched): `disabled`, `starting` (waiting for a first reading), `holding` (within tolerance), `moving`, `sensorFault`, `tuning`.

**Programmatic API** (for applications such as the sun tracker): `setTarget(deg)` (through the slew), `setTargetNow(deg)` (bypasses it; the sequencer uses this), `target()`, `position()`, `enable(bool)`, `state()`. These take Motion's lock and may be called from any task.

### 4.4 Relay autotune

`RelayAutotune` (Åström–Hägglund, as in the heliostat): the setpoint is held at the start position. The output switches to ±`amp` whenever the error leaves ±`band`. After 6 crossings, the oscillation's amplitude a and period Tu give Ku = 4·amp / (π·a). Classic Ziegler–Nichols then sets kp = 0.6·Ku, ki = kp / (0.5·Tu), kd = kp · 0.125·Tu.

- It gives up after 15 s (`failed`), and on a sensor fault.
- `start(amp, band)` and `cancel` are actions. `status` (`idle`, `running`, `done`, `failed`), `ku` and `tu` are read-only. On `done`, the new gains are written to `tuning` and persisted like any write.
- The heliostat stores I = 1/Ti and D = Td, which are not gains (§4.6). The formulas above are the correct ones, so tuned values differ from the heliostat's. Confirm them on the simulated plant (§11) and then on hardware.

### 4.5 Defaults

| Setting | Default | Range |
|---|---|---|
| `kp`, `ki`, `kd` | 0.05, 0, 0 | 0–10, 0–10, 0–10 |
| `tolerance` | 0.1° | 0–10 |
| `maxSpeed` | 0 (off) | 0–360 °/s |
| `ramp` | 0 (off) | 0–100 /s |
| `integralLimit` | 1 | 0–1 |
| `derivativeFilter` | 0.2 | 0–1 |
| `maxDerivative` | 500 °/s | 1–5000 |
| `sensorTimeout` | 200 ms | 20–5000 |
| `minOutput`, `shape` | 0.15, 1 | 0–1, 0.1–5 |
| `frequency` | 15 000 Hz | 1000–40 000 |

### 4.6 Defects of the heliostat's code not to port

| Today | TesserMotion |
|---|---|
| `Servo_Driver::setAngle()` calls `run()` from the HTTP handler's task, concurrently with the control task. | Setters record; only the control task drives (§3.5). |
| Disabling an axis leaves the motor at its last output (the `setSpeed(0)` is commented out). | Disabled means output 0 (§4.3). |
| Autotune writes I = 1/(0.5·Tu) and D = 0.125·Tu, not gains. | ki = kp/Ti, kd = kp·Td (§4.4). |
| The encoder rejects a reading of 0 (`value > 0`), so true 0° counts as an error. | Faults come from bus errors and timeouts only. Check on hardware whether a stuck MT6701 returns 0, and if it does, detect that some other way. |
| `S` gain stored, never used. | Removed. |
| Limits shifted by the offset setter (depends on load order). | Limits stored in the raw frame (§4.3). |
| `double` everywhere, software floating point. | `float` (§2). |
| Teleplot UDP telemetry, `Serial.printf` profiling. | Watched values with charts (§9.1), `Motion` statistics (§3.2). |

## 5. Sequencer

A `Sequencer` plays keyframes: one or more **tracks**, each a list of frames `{t, value}` (t in ms from the start), interpolated linearly. It replaces three classes of the heliostat: `MovementSequencer` (one axis), `TargetSequencer` (a target's azimuth and elevation) and `SourceTargetSequencer` (four values). Those differ only in what they write to.

```cpp
Sequencer azSequence(1);                  // one track, bound to an axis
azSequence.bind(0, azimuth);              // setTargetNow() each tick

Sequencer targetSequence(2, {"azimuth", "elevation"});
targetSequence.onFrame([](const float* v, size_t n) {   // NaN: track empty
    heliostat.setTarget(currentTarget, v[0], v[1]);
});
```

- **Tracks:** up to 4, of up to `maxFrames` frames each (16 by default, the heliostat's). A track is bound to an `Axis` with `bind(i, axis)`, or to a function with `bind(i, fn(float))`. Alternatively `onFrame(fn)` gets every track's value at once, for a sink that needs them together. An empty track yields NaN, and the sink keeps its current value, as `TargetSequencer` does today.
- **Timing:** `start` records the start time from the Motion clock. Each tick, a track's value at the elapsed time is the first frame's before it, the interpolation between the two frames around it, then the last frame's. The duration is the latest frame's t over all tracks. At the end, a one-shot sequence writes the last values and stops, and a looping one wraps (elapsed modulo duration). A zero duration writes the last values once.
- **Wrapping tracks:** `wrap(i, 360)` interpolates a track along the shortest arc (350° → 10° passes 0°). The default is off, as in the heliostat. An axis's limits still clamp what it receives.
- **Frames** are kept sorted by t. A write of the frames list sorts it, and two frames with the same t keep their order.
- **Ownership:** while a sequence drives an axis, a write to that axis's `target` through the tree is refused with `busy` ("a sequence is running"). `stop` releases it. `Axis::setTarget()` from code isn't refused: the application decides.
- **Start at a wall-clock time** is TesserKIT's Timers (`path` `/azimuth/sequence/start`), or NowTP's shared clock *(later)*.

## 6. Sensors

### 6.1 Accelerometer

```cpp
class Accelerometer {
public:
    virtual bool begin() = 0;
    virtual bool read(float& x, float& y, float& z) = 0;   // m/s²
};
```

- **`Adxl345`** (I2C, address 0x53 or 0x1D): checks DEVID 0xE5, then sets 100 Hz (BW_RATE 0x0A), full resolution ±16 g (DATA_FORMAT 0x0B) and measure mode (POWER_CTL 0x08). It reads six bytes from 0x32, at 3.9 mg/LSB. These are the heliostat's settings.
- **Sampling:** the accelerometer is read every `every` ticks (default 5, so 10 Hz at 50 Hz), averaged over the last `samples` readings (default 8, up to 32). It reports `x`, `y`, `z`, then `pitch` and `roll` in degrees computed from them, and `present`.
- **`average(n, fn)`**: a deferred action that collects n fresh readings and replies with their mean. This is what calibration routines use (the heliostat's `AccelCalibService` samples it this way).
- Bias and scale calibration of the sensor itself *(later)*. The heliostat's joint solver over encoder offsets and mount misalignment stays in the heliostat.

### 6.2 GPS

- **`NmeaGps`** on a `Uart` (default 9600 baud, the heliostat's NEO module). It parses `RMC` and `GGA` sentences, any talker (`GP`, `GN`...), with checksums verified. Lines that fail are counted and dropped. The parser is its own host-tested class (`Nmea`), replacing TinyGPSPlus and TimeLib.
- **Reports:** `fix` (`none`, `gps`, `dgps`, ... GGA's quality), `satellites`, `hdop`, `latitude`, `longitude` (degrees), `altitude` (m), `utc` (ISO 8601), and `age` (ms since the last valid fix).
- **Time:** `onTime(fn(int64_t epochMs))` is called on each valid RMC with a date, at most once a second. An application on TesserKIT connects it to `clock.external()` (TesserKIT §5.3).
- The UART is read every tick without blocking. Parsing happens in the control task, which is cheap at 9600 baud.
- UBX configuration (rate, constellations) and PPS *(later)*.

## 7. Drivers on ESP-IDF

### 7.1 I2C

ESP-IDF refuses to link the legacy I2C driver (`driver/i2c.h`) and the new one (`driver/i2c_master.h`) in one image: it aborts at boot with "CONFLICT! driver_ng is not allowed to be used with the legacy driver". Arduino's `Wire` uses one of them, depending on the Arduino-ESP32 version. TesserMotion uses `i2c_master`. Before writing the driver, check which one `Wire` uses in the pioarduino version TesserAPI pins (TesserAPI's AGENTS.md). If it's the legacy driver, raise it with Leo before going further: the heliostat sketch must not mix them.

- `I2cBus` owns an `i2c_master_bus_handle_t` and adds device handles per address on first use. 400 kHz by default; after `recover()`, 100 kHz, as in the heliostat.
- Transfers use a 20 ms timeout, so a stuck bus costs at most one tick.

### 7.2 LEDC

- `Pwm` allocates LEDC timers and channels from the top of the range downwards (timer 3, channels 7, 6, ...). Arduino's `analogWrite` and `ledcAttach` allocate from the bottom, so the two rarely collide. Channels can be given explicitly.
- Two motors at the same frequency share a timer.

### 7.3 UART

`Uart` installs `uart_driver_install` with a 256-byte receive buffer on the port given (default UART 1), with no event queue.

## 8. Persistence

Settings are persisted with TesserAPI's `.persist()` (TesserAPI §12): an axis's `tuning`, `limits`, `sensor/config`, `motor/config` and `enabled`; a sequencer's frames and `loop`; an accelerometer's `samples`. Status values (`position`, `state`, ...) aren't. As TesserAPI loads leniently, a firmware with a renamed setting boots with its default.

## 9. The trees

Paths below are relative to where the application mounts each part. `(p)` is persisted, `(w)` watched. All angles in degrees with `.unit("°")`.

### 9.1 Axis

```
azimuth/                          label from the application
  position      number   (w)      read-only, step 0.01, ui chart
  target        number            0..360, step 0.01; busy while a sequence drives it
  error         number   (w)      read-only
  output        number   (w)      -1..1, read-only, ui chart
  state         enum     (w)      disabled, starting, holding, moving, sensorFault, tuning
  enabled       boolean  (p)
  stop          action            the setpoint becomes the position: stops a slew, holds
  motor/
    output      number            -1..1; writable only while the axis is disabled (else not_allowed)
    duty        integer  (w)      0..100 %, read-only
    config/     (p), ui collapsed
      invert, minOutput, shape, frequency
  sensor/
    raw         number   (w)      read-only
    faults      integer           read-only: failed reads since boot
    config/     (p), ui collapsed
      invert, offset (-360..360)
  limits/       (p)
    enabled, begin, end           begin and end 0..360; visibleWhen enabled
  tuning/       (p), ui collapsed
    kp, ki, kd, tolerance, maxSpeed, ramp, integralLimit, derivativeFilter, maxDerivative, sensorTimeout
  autotune/     ui form
    amp (0.05..1), band (0.1..10)
    start, cancel                 actions
    status      enum              idle, running, done, failed
    ku, tu      number            read-only
```

`position`, `target`, `error` and `output` have `ui.widget "chart"`, which replaces Teleplot. TesserUI keeps their history, so the device stores nothing (TesserUI §7.7).

### 9.2 Sequencer

The shape of TesserUI's `x-keyframes` (TesserUI §9.8): one list of `{t, value}` per track, named after the track (`frames` for a single unnamed track).

```
sequence/                         ui.widget x-keyframes (ui.script set by the application)
  frames        list (p)          {t: integer ms 0..86 400 000, value: number}, maxSize maxFrames
  loop          boolean (p)
  running       boolean (w)       read-only
  position      integer (w)       ms since start, read-only; the playhead
  duration      integer           ms, read-only
  start, stop, clear              actions
```

A track bound to an axis takes the axis's range for `value`, so the element bounds its curve. A track bound to a function has the range given by `range(i, lo, hi)`.

### 9.3 Sensors

```
tilt/                             Adxl345
  x, y, z       number (w)        m/s², read-only
  pitch, roll   number (w)        °, read-only
  present       boolean
  samples       integer (p)       1..32
  average       action            deferred; argument n (1..256), replies {x, y, z}

gps/                              NmeaGps
  fix           enum   (w)
  satellites    integer (w)
  hdop, latitude, longitude, altitude   number (w)
  utc           string
  age           integer (w)       ms
  errors        integer           checksum and parse failures
```

### 9.4 Motion

`motion.describe(o)` (the application mounts it where it likes, such as `/sys/motion` on TesserKIT): `period` (ms), `tickMax` (µs, reset on read), `overruns`, and `stopAll` (an action with `ui.style "danger"`).

### 9.5 From the heliostat's REST routes

For the helionext port (TesserKIT §10.1), the heliostat's `/rest/heliostat/azimuth` router becomes:

| Today | TesserMotion |
|---|---|
| `position`, `target`, `tolerance`, `offset`, `invert`, `enabled` | `position`, `target`, `tuning/tolerance`, `sensor/config/offset`, `sensor/config/invert`, `enabled` |
| `P`, `I`, `D`, `S`, `maxSpeed`, `ramp` | `tuning/kp`, `ki`, `kd`, (removed), `maxSpeed`, `ramp` |
| `curGain`, `derivative`, `integral`, `encoderError` | `output`, (internal), (internal), `state` and `sensor/faults` |
| `autotune {start, cancel, amp, band, active, done, Ku, Tu}` | `autotune/` actions and values |
| `limits {enabled, begin, end}` | `limits/` |
| `motor {control: {speed, duty, direction}, config: {minVal, shape, enable, invert}}` | `motor/output`, `motor/duty`, `motor/config/{minOutput, shape, invert}` |
| `plot` | charts |
| `…/sequence {frames: [{tMs, value}], loop, start, stop, clear, running, duration}` | `sequence/` with `t` in place of `tMs` |

## 10. Footprint

Measured as in TesserAPI §15 and TesserKIT §11: an example built with and without the parts, binary sizes subtracted, `idf.py size-files` for detail. The first measurement sets the budgets. Until then the rule is Leo's: keep features, make large ones optional, and save space only where it costs no performance, clarity or ease of use. To report, on a classic ESP32 at `-Os`:

| Build | Above TesserAPI |
|---|---|
| Two axes (TB6612, MT6701), no tree | |
| The same with their trees | |
| + two sequencers | |
| + ADXL345, GPS | |
| RAM at rest, the task's stack included | |

## 11. Testing

1. **Host tests** (CMake, ASan and UBSan, the warnings of TesserAPI's AGENTS.md), against `test/fake_hw.*`:
   - **Plant:** a geared DC motor, J·θ″ = k·u − b·θ′ − static friction, with a dead band below `minOutput`, driving a 14-bit encoder with noise. Fault injection: NACKs, timeouts, a stuck bus.
   - **Axis:** reaches and holds targets within tolerance, at 20 and 50 Hz; never crosses the forbidden arc (randomized targets and offsets); slew speed respected; sensor timeout stops the motor and recovers; no jump at start; limits unchanged by an offset change, in either load order; disabled means output 0.
   - **Autotune** finds the plant's Ku and Tu within 20 % of their analytic values, and the gains it writes hold a target.
   - **Motor:** output mapping, the TB6612 truth table on fake GPIOs (brake vs coast), invert.
   - **Sequencer:** before the first frame, between frames, after the end, looping, zero duration, empty tracks (NaN), wrap, unsorted writes, refusal of axis target writes while running.
   - **Drivers:** MT6701 and ADXL345 register traffic on the fake bus; NMEA sentences (real captures from the heliostat's module in `test/nmea/`, bad checksums, partial lines, no fix).
   - **Trees:** golden schemas in `fixtures/*.json` (axis, sequencer, tilt, gps), as in TesserKIT §12, rendered in TesserUI's golden-model tests. They replace TesserUI's hand-written `motors` fixture (a gap report to TesserUI's agent).
2. **On target:** `examples/idf/axis` (one axis, a sequencer, the accelerometer, on TesserKIT for Wi-Fi and the interface), driven by `test/e2e_motion.py` over HTTP and WebSocket: enable, move to targets and read back positions within tolerance, slew timing, limits, a sequence run, a sensor fault (unplugging is manual; the script asks), autotune, persistence across a restart. Motors move: see AGENTS.md before running it.
3. **CI:** host tests, ESP-IDF builds (5.2, 5.4, 5.5, latest × ESP32, ESP32-C3, ESP32-S3), Arduino-ESP32 3.x builds of the examples.

## 12. Packaging

```
TesserMotion/
  docs/DESIGN.md
  src/TesserMotion.h
  src/tessermotion/            drivers, controllers, *_tree.cpp
  src/tessermotion/platform/esp/
  examples/idf/axis/
  examples/arduino/TwoAxes/
  fixtures/
  test/                        host tests, fake_hw, nmea captures, e2e_motion.py
  CMakeLists.txt, idf_component.yml, library.json, library.properties
```

- ESP-IDF component and PlatformIO/Arduino library, like its siblings. Namespace `tessermotion`. TesserAPI is a hard dependency, found as TesserKIT finds it: any component name ending in `tesserapi`, and `TESSERAPI_DIR` (default `../TesserAPI`) or GitHub main for the host build.
- `PRIV_REQUIRES`: `esp_driver_gpio`, `esp_driver_ledc`, `esp_driver_i2c`, `esp_driver_uart`, `esp_timer`. These are components of ESP-IDF itself, so a project that doesn't instantiate a part pays nothing for them.
- MIT, semver, public at <https://github.com/leokeba/TesserMotion> once Leo creates it.

## 13. Later

| Item | For |
|---|---|
| Steppers: a TMC5160 driver on `spi_master` (its ramp generator, without FastAccelStepper), `StepperAxis` with encoder correction and the 128-point linearization table | spin-seq, TesserKIT phase 4 |
| Multi-turn angle sensors (unwrapped position, turns) | continuous rotation axes |
| Other encoders: AS5600 (I2C), AS5048A (SPI) | other mechanics |
| A command sequencer (spin-seq's typed commands, TesserKIT §10.2) | spin-seq |
| Accelerometer bias and scale calibration | sensors on their own |
| UBX configuration and PPS for the GPS | time accuracy |
| MCPWM motor driver with PCNT encoders (`SOC_*`-guarded) | quadrature-encoded motors |
| Synchronized starts across nodes (NowTP's shared clock) | fleets |

## 14. Roadmap

| Phase | Content | Acceptance |
|---|---|---|
| 1 | Packaging, platform interfaces and fakes, the plant model, CI-ready host build | An empty `Motion` ticks on the host and builds for ESP32, C3, S3 on ESP-IDF and Arduino. |
| 2 | `DcMotor`/`Tb6612Channel`, `Mt6701`, `Axis`, autotune, `Motion`, their trees | §11's axis and autotune tests pass; one axis of the heliostat holds targets from the web interface. |
| 3 | `Sequencer` | The heliostat's azimuth and elevation sequences run from `x-keyframes`. |
| 4 | `Adxl345`, `NmeaGps` | Tilt and GPS fix reported on the heliostat; the clock set from GPS through TesserKIT. |
| 5 | Handover | TesserKIT's phase 3 (helionext) uses TesserMotion; footprint measured (§10). |
