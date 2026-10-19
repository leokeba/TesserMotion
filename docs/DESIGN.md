# TesserMotion design

Status: phases 1 and 2 implemented (2026-10-11): the platform layer, `DcMotor`/`Tb6612Channel`, `Mt6701`, `Axis`, `Motion` and their trees, host-tested against a simulated plant, and commissioned on the heliostat's two axes (through tiny-heliostat). Phase 4's sensors (`Adxl345` with `Tilt`, `NmeaGps`) implemented, host-tested, and run on the second heliostat (2026-10-16, through tiny-heliostat): tilt, GPS fix and the clock set from GPS. The angle sensor's correction (§4.2) implemented and host-tested (2026-10-18), and the accelerometer's (§6.1, 2026-10-19); neither run on hardware yet. The Trinamic drivers (§4.7, `Tmc5160` on SPI, `Tmc2209` on UART) and their register maps generated from TMC-API (§4.8) implemented and host-tested against simulated chips (2026-10-19). The TMC2209 run on Fluante's board (2026-10-19, through fluante-firmware, ESP-IDF 6.1-dev): found at every boot once the UART waited whole ticks (§7.3), the motor turned through `VACTUAL` (at 30 and 116 rpm of the sculpture, `TSTEP` within 1 % of the velocity asked), stealthChop, enabling and the deadman's renewal as specified. The TMC5160 hasn't run on hardware yet. This is the specification the code follows; when the two disagree, fix one of them. Sections marked *(later)* are committed to but not implemented yet. References to TesserAPI, TesserUI and TesserKIT point to their own DESIGN.md: [TesserAPI](https://github.com/leokeba/TesserAPI/blob/main/docs/DESIGN.md), [TesserUI](https://github.com/leokeba/TesserUI/blob/main/docs/DESIGN.md), [TesserKIT](https://github.com/leokeba/TesserKIT/blob/main/docs/DESIGN.md).

## 1. Goals

TesserMotion is a library of motion control and sensing for ESP32 devices built on [TesserAPI](https://github.com/leokeba/TesserAPI): motor drivers, encoders, closed-loop axes, keyframe sequencers, and the sensors that go with them (accelerometer, GPS). Every part describes itself into a TesserAPI tree, so it is reachable over every transport and drawn by [TesserUI](https://github.com/leokeba/TesserUI) without UI code. Its first user is the heliostat (`~/Dev/open-heliostat`, branch `helionext`), ported to [TesserKIT](https://github.com/leokeba/TesserKIT) in TesserKIT's phase 3. Its second will be the spin sequencer (`~/Dev/spin-sequencer`), once steppers are added (§13).

- **A library, separate from TesserKIT.** TesserKIT holds what every connected device needs. Motion is what some devices do. A TesserKIT device without motors never compiles or downloads this code, and TesserMotion doesn't depend on TesserKIT (§3.4).
- **Pay for what you use.** A part that isn't instantiated isn't linked: no static registration, no feature flags. Chip-specific drivers compile only on chips that have the peripheral.
- **Three layers per part.** A driver talks to hardware. A controller holds the logic and is host-tested. A tree adds the TesserAPI description. Each layer is usable without the ones above it (§3.1).
- **Zero UI code.** Parts carry TesserUI hints. The keyframe editor is TesserUI's `x-keyframes` example element (TesserUI §9.8), whose tree shape the sequencer follows exactly.
- **Host-tested against a simulated plant.** The PID, limits, slew, autotune and sequencer run on the host against a model of a geared DC motor with a quantized, noisy encoder (§11).
- **The heliostat's behaviour, minus its bugs and its clutter.** The port keeps what the heliostat needs to hold and move its mirror (§4.3), fixes the defects listed in §4.6, and leaves out what nothing uses. Features come when an application needs them, not before.

### Non-goals

- Application geometry: sun position, mount orientation solving, the joint accelerometer and encoder calibration, sources and targets. They stay in the heliostat.
- Stepper axes and step generation in the first release (§13). The Trinamic drivers (§4.7) are in: they configure the chips, and turn the motor through the chips' own generators.
- Motion planning beyond one axis: synchronized multi-axis moves come from a sequencer with one track per axis (§5), not from a planner.

## 2. Constraints

| Item | Decision |
|---|---|
| Language | C++17, TesserAPI's rules: no exceptions, no RTTI, never `throw`/`dynamic_cast`/`typeid`; calling an empty `std::function` aborts, so check it first. Virtual functions are fine. |
| Frameworks | ESP-IDF ≥ 5.2 and Arduino-ESP32 ≥ 3.2 (whose `Wire` uses the I2C master driver too, §7.1). The same code on both: ESP-IDF drivers only, no Arduino APIs (no `Wire`, `ledcWrite`, `HardwareSerial`, `millis`). |
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
                   ├─▶ Tilt ──▶ Accelerometer (Adxl345 on I2cBus, shareable with an Mt6701)
                   ├─▶ NmeaGps (Nmea on Uart)
                   └─▶ TmcDriver (Tmc5160 on Spi, Tmc2209 on Uart)

 tesser::Api ◀── describe(Object&, Axis&), describe(Object&, Sequencer&), ...
```

### 3.1 Layers

| Layer | Content | Depends on |
|---|---|---|
| Driver | `Tb6612Channel`, `Tb6612Standby`, `Mt6701`, `Adxl345`, `NmeaGps`, `Tmc5160`, `Tmc2209`: register maps, pin logic, scaling. | The platform interfaces (§3.3). |
| Controller | `DcMotor` output mapping, `Axis` (with its PID), `RelayAutotune`, `Sequencer`, `Tilt`, `Motion`; the `Nmea` parser. Pure logic; time comes in as an argument. | Drivers through their interfaces (`AngleSensor`, `DcMotor`, `Accelerometer`). |
| Tree | `describe()` free functions, found by argument-dependent lookup as TesserAPI's `mount()` expects (TesserAPI §4.3), in their own `*_tree.cpp` files. | TesserAPI. |

Headers of the first two layers don't include TesserAPI, so a project can use an `Axis` without a tree. The package still lists TesserAPI as a dependency.

### 3.2 The control task

```cpp
#include <TesserMotion.h>
using namespace tessermotion;

Tb6612Standby bridge(18);
Tb6612Channel azMotor(bridge, {.pwm = 16, .in1 = 17, .in2 = 5});
I2cBus bus1({.sda = 13, .scl = 14}), bus2({.sda = 27, .scl = 26});
Mt6701 azSensor(bus1);
Axis azimuth(azSensor, azMotor);
Motion motion;

void setup() {
    motion.add(azimuth);
    api.mount("azimuth", azimuth);
    api.object("sys").mount("motion", motion);
    // ... load the stored settings ...
    motion.start(50);   // Hz; then a TaskConfig: name, stack, priority, core
}
```

- **Parts.** Everything the task drives is a `Part`: `begin()`, `sense(nowUs)` (reads sensors, outside the lock), `update(nowUs)` (computes and applies outputs, under the lock) and `stop()` (for `stopAll()`). `Axis` is one; an application adds its own, such as a sun tracker that sets the axes' targets each tick.
- **One task, fixed rate.** `Motion::start(rateHz, task)` calls `begin()`, then starts a task that calls `step(nowUs)` every period with `vTaskDelayUntil`, so the period is rounded to the RTOS tick (10 ms on ESP-IDF's default 100 Hz tick). Defaults: 50 Hz, priority `configMAX_PRIORITIES - 2`, core 1 on dual-core chips, 4 KB stack. The heliostat runs at 20 Hz today; 50 Hz is the new default, to be confirmed on hardware (§11).
- **Tick order:** every part's `sense()`, then every part's `update()`, both in the order the parts were added. So add sequencers and trackers before the axes they drive.
- **`step(nowUs)` is public.** Tests call it directly with a fake clock, without a task. So does an application that wants to drive the loop itself (it calls `begin()`, never `start()`).
- **Constructors do nothing but store arguments** (the static-initialization rule, TesserKIT's AGENTS.md). Hardware setup happens in `Motion::begin()`, which calls each part's `begin()` in the order they were added.
- **Statistics:** the task measures its period and the duration of each tick. `Motion` reports `period` (ms, the last), `tickMax` (µs since the last read) and `overruns` (ticks that took longer than the period) as read-only values (§9.4). These replace the heliostat's "Servo loop lag" and "Control loop slow" prints.
- **`stopAll()`** disables every axis and zeroes its manual output (and later stops every sequencer), from any task. An application calls it before a firmware update (TesserKIT's `dev.onState()`, state `Updating`) and from its own emergency paths.

### 3.3 Platform layer

Narrow interfaces in `src/tessermotion/platform.h`, an ESP implementation in `src/tessermotion/platform/esp/`, and fakes in `test/fake_hw.*`. A part takes the interfaces it needs, so an interface without a user isn't linked.

| Interface | Methods | ESP implementation |
|---|---|---|
| `Gpio` | `output(pin, level)`, `set(pin, level)` | `driver/gpio.h` |
| `Pwm` | `attach(pin, frequencyHz) → channel`, `duty(channel, fraction 0..1)` | LEDC (§7.2) |
| `I2c` | `begin()` (idempotent: every device on the bus calls it), `read(addr, reg, buf, len)`, `write(addr, reg, buf, len)`, `recover()` | `I2cBus`, on `driver/i2c_master.h` (§7.1) |
| `Spi` | `begin()` (idempotent), `transfer(device, tx, rx, len)`, a device being its chip select, mode and clock | `SpiBus`, on `driver/spi_master.h` (§7.4) |
| `Uart` | `begin()`, non-blocking `read(buf, max) → n`, `read(buf, len, timeoutMs) → n` (waits for `len` bytes), `write(buf, len)` | `UartPort`, on `driver/uart.h` (§7.3) |
| `Lock` | `lock()`, `unlock()`; recursive | a static FreeRTOS recursive mutex; `std::recursive_mutex` on the host |
| `startPeriodic(task, periodUs, tick)`, `nowUs()` | free functions: the control task and the clock | `xTaskCreatePinnedToCore` with `vTaskDelayUntil`; `esp_timer_get_time()` |

- **Defaults:** a TB6612 takes a `Gpio*` and a `Pwm*`, null by default, meaning the chip's (`defaultGpio()`, `defaultPwm()`). Tests pass fakes, or use the defaults, which the fakes provide on the host.
- **`UartPort`** is a concrete class on ESP too: `UartPort uart({.rx, .tx, .baud, .port})`, 9600 baud and UART 1 by default.
- **`SpiBus`** is a concrete class on ESP (`SpiBus spi({.sclk, .mosi, .miso, .host})`, SPI2 by default) that implements `Spi`. `SpiBus(host)` shares a host other code initialized (an Ethernet chip's).
- **`I2cBus`** is a concrete class on ESP (`I2cBus bus({.sda, .scl, .port, .hz})`) that implements `I2c`. It can also wrap a bus handle the application already created (`I2cBus bus(existingHandle)`), so TesserMotion shares a bus with other code; Arduino's `Wire` gives its handle with `i2cBusHandle(num)`.

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

- **`Motion` has one lock of its own** (`Lock`, §3.3), which `Motion::add()` gives each part. The control task holds it while parts compute and apply outputs, which is tens of microseconds, never during bus transfers. Tree getters and setters take it around the fields they touch. It is recursive, so a part's `update()` may call another part's public methods, which take it too.
- **Sensor reads happen before the lock.** A sensor's result is stored in the part, then picked up under the lock. A slow or failing I2C transfer (up to its timeout) therefore never blocks a tree request.
- **Lock order:** the API lock, then Motion's lock. The control task never takes the API lock. Status values reach subscribers because TesserAPI samples watched getters under its own lock, and those getters take Motion's lock briefly.
- **Only the control task drives hardware.** A tree write records a request (a new target, enable, a manual output), and the next tick applies it. No setter calls a motor or a sensor. This removes the heliostat's race, where `setAngle()` called `run()` from the HTTP task while the control task was running (§4.6).

## 4. Motors and axes

### 4.1 DC motors and the TB6612

```cpp
class DcMotor {                         // what an Axis drives
public:
    virtual bool begin() = 0;
    void output(float u);               // -1..1, sign is direction
    bool invert; float minOutput, shape;   // settings
protected:
    virtual void drive(bool forward, float duty) = 0;   // the bridge's part
};
```

`DcMotor` implements the output mapping and holds the settings; a bridge driver derives from it. `Tb6612Channel` is a `DcMotor` on one channel of a TB6612FNG (PWM, IN1, IN2), sharing a `Tb6612Standby` (the STBY pin) with the other channel. Steppers will be another kind of axis (§13), not another motor behind `Axis`, so there is no abstract motor interface.

- **Output mapping** (the heliostat's, kept): for |u| ≥ 0.001, duty = |u|^`shape` × (1 − `minOutput`) + `minOutput`; below that, 0. `minOutput` compensates the motor's dead band, and `shape` curves the response. `invert` swaps the direction. u is clamped to ±1, and NaN is 0.
- **TB6612 truth table:** IN1 H / IN2 L forward, L / H reverse. PWM low gives a short brake between pulses (the chip's slow decay). `output(0)` keeps the direction pins and sets duty 0, which brakes: the heliostat's behaviour, which holds a mirror against wind. Coasting (IN1 L / IN2 L) isn't offered: nothing needs it.
- **Standby:** a channel's `begin()` sets its direction pins and PWM, then calls `Tb6612Standby::begin()`, which drives STBY high once. If the PWM can't be attached, the channel stays off and STBY low. `stopAll()` and a sensor fault don't touch STBY; `Tb6612Standby::enable(false)` is for the application.
- **PWM:** 15 kHz by default (the heliostat's), set in `Tb6612Pins::frequencyHz` at construction. The LEDC resolution is the highest the frequency allows on the chip (12 bits at 15 kHz on the ESP32).

### 4.2 Angle sensors

```cpp
class AngleSensor {
public:
    virtual bool begin() = 0;
    virtual bool read(float& degrees) = 0;   // 0..360, raw; false on a bus error
};
```

`Mt6701` reads an MT6701 magnetic encoder over I2C (address 0x06): a 14-bit angle in registers 0x03 (bits 13–6) and 0x04 (bits 7–2 hold bits 5–0). The heliostat reads three bytes from 0x02 and uses the last two; TesserMotion reads two from 0x03, which should give the same values. Check that on hardware, and go back to the three-byte read if it doesn't. Each MT6701 has the same fixed address, so two encoders need two buses (the heliostat uses I2C 0 and 1).

**Correction.** An angle sensor's error repeats every turn: an off-centre or tilted magnet, an uneven field. `AngleCorrection` models it as a short Fourier series in the raw angle, corrected = raw + c(raw), with c(θ) = Σ a_k·cos kθ + b_k·sin kθ for k = 1..K (K ≤ 6). Eccentricity is mostly k = 1, with some k = 2; tilt and the magnet add k = 2 and 3.

- **Points** are pairs: the raw reading, and a reference, what a perfect sensor would have read. Where they come from doesn't matter: a protractor or a jig with a DC motor, steps counted with a stepper, the accelerometer, the sun. Each has a weight (1/σ², so rough and accurate references mix); weight 0 leaves a point out, and a point recorded several times counts as their average.
- **The fit** is weighted least squares for a₀ and the 2K coefficients, with a ridge on the coefficients (10⁻³ per point, with the weights scaled to average 1). The ridge doesn't change the result over a full turn of points, but it keeps the correction small away from the points on a short arc: over 90° (an elevation axis), 10 points cut a 0.28° error to 0.015° there, while the correction stays below 0.6° on the rest of the circle. K = 0 fits as many as the points support (2K + 1 ≤ points), up to 3. A fit fails, keeping the correction as it was, without enough points for the harmonics asked, with a point that isn't a number, or when the result would be too steep to invert (|c′| ≥ 0.5, references far off: 2° of first harmonic is 0.035).
- **a₀,** the constant between the references and the readings, is fitted but not applied: the sensor's `offset` stays the zero, so references may have any zero (a stepper's count from wherever it started). On an arc short of a turn, part of the error can't be told from a constant, and goes into a₀ too. The residuals report it, as what to add to the offset when the references were absolute.
- **Residuals,** computed on each read from the points and the correction as it is (so also before any fit, uncorrected): the points used, `zero` (a₀), the RMS and the maximum of what is left once the zero is taken out, and the arc the points cover.
- **Applying** is in the axis, between the raw and the output frames (§4.3). The control loop never evaluates it: a reading is corrected only to show it, and a target is converted once, by the fixed-point steps raw ← corrected − c(raw).

**Bus faults:** after 3 failed reads in a row (a NACK, a bus error or a 20 ms timeout, §7.1), the sensor calls `bus.recover()` and counts again from zero, so a dead bus is recovered every third tick at most. The axis sees `read()` return false and counts it (§4.3, §9.1). The heliostat's separate 100 ms cooldown and slow-transfer trigger are gone: the count spaces recoveries, and a slow transfer is a timeout.

### 4.3 Axis

An `Axis` is a closed-loop position servo: an `AngleSensor`, a `DcMotor`, and a PID between them, in degrees on a circle.

- **Two frames.** The raw frame is the sensor's reading. The output frame is what the tree and the API show and take: position = (−r if `sensor.invert`, else r) + `sensor.offset`, modulo 360, where r = raw + c(raw) is the corrected reading (§4.2). Internally, the target, the setpoint and the limits are all in the raw frame. So changing the offset, the sensor's `invert` or its correction (a calibration) never moves the motor, and loading persisted settings gives the same result in any order. The heliostat ran its PID in the output frame, so a new offset moved the axis and the encoder's `invert` changed the control's sign.
- **Direction:** a positive motor output must increase the raw angle; the motor's `invert` sets that. The sensor's `invert` only chooses which way the output frame turns. (A heliostat axis whose encoder was inverted needs its motor's `invert` flipped once when its settings are carried over.)
- **Limits are physical:** an arc of the raw frame, from `begin` to `end` in increasing angles, given as `sensor/raw` reads them (jog the axis to each stop and note the reading). `begin == end` means no limits. A target outside the arc is clamped to the nearer end, measured from the arc's middle. With limits on, the path to the target is measured from the arc's middle, so the axis never moves through the forbidden side. Without limits, it takes the shortest way around.
- **Slew:** with `maxSpeed` > 0 (°/s), the setpoint moves towards the target at that speed along the allowed path, one step per tick. A change of `maxSpeed` applies from where the setpoint is. With 0, the setpoint is the target.
- **PID**, on the error e from the position to the setpoint (degrees, along the allowed path), with the measured period dt:
  - integral += e·dt, clamped to ±`integralLimit` / ki (so the integral term alone never exceeds `integralLimit` of full output); 0 while ki is 0;
  - velocity = low-pass(`derivativeFilter`) of the measured dposition/dt. Derivative on measurement, so a jump of the setpoint gives no kick;
  - u = kp·e + ki·integral − kd·velocity, clamped to ±1;
  - inside `tolerance`, u = 0 and the integral resets. The axis is `holding` when inside tolerance with its setpoint at the target.
- **Sensor timeout:** a tick without a reading keeps the last output. Without a good reading for `sensorTimeout` (200 ms), the axis outputs 0 and its state is `sensorFault`. It resumes when readings come back, towards the same target.
- **Enabling** goes through `starting`: the axis waits for a good reading, sets its target and setpoint to the position (so it never jumps), then runs. Axes boot disabled; an application that wants them running enables them in its setup.
- **Disabled:** the motor gets the manual output (`motor/output`, 0 by default: a brake), and the target follows the position. The manual output is a deadman: not rewritten for `deadman` (1 s), it drops to 0, so a client that never sends the 0 (a dropped connection, a closed tab, a killed script) can't leave the motor turning. A client that drives for longer repeats the write; the tree hints `ui.keepalive` 400 ms, so TesserUI re-sends it while shown.
- **Jog** (closed loop, for positioning by hand): `jog(direction)` moves the target and the setpoint together at `jogSpeed` (5 °/s), towards increasing or decreasing output-frame angles, clamped to the limits. It's a deadman too: not renewed within `deadman`, it stops. `jog(0)`, `setTarget()`, `hold()`, a disable and a sensor fault also stop it, and the axis holds where the jog left it. Refused unless the axis is running; the tree's `jogMinus` and `jogPlus` are also refused `busy` while the axis is claimed, with the same "driven by <owner>" message as `target`. Enabling or disabling resets the manual output to 0. Target writes are refused while the axis isn't running (`disabled` or `starting`), so a target set before enabling can't make it jump; manual output is refused while it's enabled.
- **Ownership:** `claim(owner)` marks the target as driven by something (a sequence, a tracker); until `release()`, tree writes to `target` are refused with `busy`, "driven by <owner>". Code isn't refused: the application decides.

**States** (an enum, read-only, watched): `disabled`, `starting` (waiting for a first reading), `holding` (at the target, within tolerance), `moving`, `sensorFault`.

**Calibration:** `calibrationPoint(reference, point)` makes a point at the current reading, which should show `reference` (output frame). `fitCorrection(points, n, harmonics)` fits points whose references are in the output frame, with the offset and `invert` as they are now, and applies the result (nullptr, else why); `correctionResiduals(points, n)` reports how well the correction explains them, with `zero` in the output frame. References are in the output frame so that `zero` is what the offset still lacks, and goes to 0 once added. Flipping the sensor's `invert` after recording turns them the wrong way: record them again. The tree keeps its points in `calibration` (a public field, under the API lock once mounted).

**Programmatic API** (for applications such as the sun tracker): `setTarget(deg)` (through the slew), `setTargetNow(deg)` (the setpoint jumps; sequencers and trackers that already move smoothly), `hold()` (stops whatever the axis does: a slew, a jog, a manual output), `target()`, `position()`, `error()`, `output()`, `raw()` (uncorrected), `hasReading()`, `faults()`, `enable(bool)`, `enabled()`, `state()`, `setManualOutput(u)`, `claim(owner)`, `release()`, `owner()`. They take Motion's lock and may be called from any task, a part's `update()` included. The settings are public fields (`tuning`, `sensorConfig`, `correction`, `limits`; the motor's `invert`, `minOutput`, `shape`): set them before `Motion::start()`, or under `axis.lock()` afterwards, as the tree does.

### 4.4 Relay autotune *(later)*

When an application asks for it: an opt-in part, not described in the tree and not linked unless used. `RelayAutotune` (Åström–Hägglund, as in the heliostat): the setpoint is held at the start position. The output switches to ±`amp` whenever the error leaves ±`band`. After 6 crossings, the oscillation's amplitude a and period Tu give Ku = 4·amp / (π·a). Classic Ziegler–Nichols then sets kp = 0.6·Ku, ki = kp / (0.5·Tu), kd = kp · 0.125·Tu.

- It gives up after 15 s (`failed`), and on a sensor fault.
- `start(amp, band)` and `cancel` are actions. `status` (`idle`, `running`, `done`, `failed`), `ku` and `tu` are read-only. On `done`, the new gains are written to `tuning` and persisted like any write.
- The heliostat stores I = 1/Ti and D = Td, which are not gains (§4.6). The formulas above are the correct ones, so tuned values differ from the heliostat's. Confirm them on the simulated plant (§11) and then on hardware.

### 4.5 Defaults

| Setting | Default | Range |
|---|---|---|
| `kp`, `ki`, `kd` | 0.05, 0, 0 | 0–10, 0–10, 0–10 |
| `tolerance` | 0.1° | 0–10 |
| `maxSpeed` | 0 (off) | 0–360 °/s |
| `integralLimit` | 1 | 0–1 |
| `derivativeFilter` | 0.2 | 0.01–1 |
| `sensorTimeout` | 200 ms | 20–5000 |
| `jogSpeed` | 5 °/s | 0.1–90 |
| `deadman` | 1000 ms | 500–10 000 (above the 400 ms keep-alive) |
| `minOutput`, `shape` | 0.15, 1 | 0–1, 0.1–5 |
| PWM frequency | 15 000 Hz | set at construction |

### 4.6 Defects and clutter of the heliostat's code not to port

| Today | TesserMotion |
|---|---|
| `Servo_Driver::setAngle()` calls `run()` from the HTTP handler's task, concurrently with the control task. | Setters record; only the control task drives (§3.5). |
| Disabling an axis leaves the motor at its last output (the `setSpeed(0)` is commented out). | Disabled means the manual output, 0 by default (§4.3). |
| Autotune writes I = 1/(0.5·Tu) and D = 0.125·Tu, not gains. | ki = kp/Ti, kd = kp·Td (§4.4). |
| The encoder rejects a reading of 0 (`value > 0`), so true 0° counts as an error. | Faults come from bus errors and timeouts only. Check on hardware whether a stuck MT6701 returns 0, and if it does, detect that some other way. |
| `S` gain stored, never used. | Removed. |
| Limits shifted by the offset setter (depends on load order); the PID runs in the output frame, so an offset change moves the axis. | Everything internal in the raw frame (§4.3). |
| An output ramp, a derivative clamp, and a derivative mixing error and measurement. | Derivative on measurement, low-pass filtered. The rest removed until a mechanism needs it. |
| `double` everywhere, software floating point. | `float` (§2). |
| Teleplot UDP telemetry, `Serial.printf` profiling. | Watched values with charts (§9.1), `Motion` statistics (§3.2). |

### 4.7 Trinamic stepper drivers

```cpp
SpiBus spi({.sclk = 18, .mosi = 23, .miso = 19});
Tmc5160 stepper(spi, {.cs = 5, .enable = 4});            // a Part: motion.add(stepper); api.mount("stepper", stepper);
UartPort tmcUart({.rx = 16, .tx = 17, .baud = 115200});
Tmc2209 small(tmcUart, {.address = 0, .enable = 21});
```

`TmcDriver` holds what the chips share; `Tmc5160` (SPI, its own ramp generator) and `Tmc2209` (single-wire UART, a step and direction driver) add their transport, their current scaling and their motion. They are drivers, not axes: they configure the chip, report its state, and turn the motor through the chip's own generators. A stepper axis in degrees, a step generator for step and direction inputs, and a calibration against an encoder come later (§13).

- **A Part.** The control task makes every transfer. Each tick, `sense()` writes what changed (up to 16 registers on SPI, 4 on the UART, where a write takes 0.7 ms at 115200 baud), then reads the chip's state: the TMC5160's `GSTAT`, `DRV_STATUS`, `XACTUAL` and `VACTUAL`; the TMC2209's `DRV_STATUS`, about 1.2 ms. It also reads one other readable register in turn, so the register list (§4.8) stays fresh: every readable register within 22 ticks on the TMC5160 (0.44 s at 50 Hz), 14 on the TMC2209. Requests from other tasks (settings, motion, register writes) reach the chip on the next tick.
- **A copy of every register.** The driver keeps what it last read (readable registers) or wrote (write-only ones). Each tick, under the lock, it sets the bits its settings and its motion own in that copy, and a register whose value changed is written. A register written through `writeRegister()` keeps its other bits; the owned ones follow their setting again on the next tick. Writes go in increasing addresses, which on the TMC5160 puts `RAMPMODE` before `VMAX` before `XTARGET`. A register about to be written isn't overwritten by a read in between.
- **Configuration**, at `begin()` and whenever the chip is found again: the driver reads the readable registers nothing wrote (its status, its position), writes TMC-API's sample configuration (the TMC5160's `GCONF`, `SHORT_CONF`, `DRV_CONF`, `IHOLD_IRUN`, `TPOWERDOWN`, `VSTOP`, `ENC_CONST`, `CHOPCONF`; the TMC2209's `GCONF`, `IHOLD_IRUN`, `TPOWERDOWN`, `CHOPCONF`, `PWMCONF`) under every register written since boot, applies the settings, and clears `GSTAT`.
- **Absent and reset chips.** A chip found by its version in `IOIN` (0x30, 0x21) is present. Three failed reads in a row make it absent (an SPI reply of all ones, a UART reply missing, short or with a bad CRC, a version read in turn that isn't the chip's), and it is looked for every 2 s, as the tilt's sensor is. Every failed transfer counts in `errors`, searches for an absent chip included, and `lastError` says why the last one failed: `bus` (the bus refused it), `noReply`, `echoOnly` (the UART's own request came back, no reply: wiring fine, the chip silent or at another address), `badReply` (short, or a bad sync, register or CRC), `wrongVersion`. `GSTAT.reset` read set means the chip lost power (its motor supply, on boards that power the logic from it): it is counted (`resets`) and configured again, disabled. A reset chip lost its position, so the driver never resumes a motion by itself.
- **Enabling:** drivers boot disabled. Disabled is `TOFF` 0 (the power stage off, the motor free) and the enable pin high, if one is given (DRV_ENN, active low; -1 for a board that ties it low). Enabling or disabling stops any motion, and `stopAll()` disables. The tree refuses enabling an absent driver.
- **Settings,** in physical units: `runCurrent` and `holdCurrent` (A rms; the hold current at most the run current, `IHOLDDELAY` and `TPOWERDOWN` from the sample configuration), `microsteps` (1 to 256, a power of 2: `MRES`), `interpolate` (to 256 microsteps, `intpol`), `invert` (`shaft`), `stealthChop` (the TMC5160's `en_pwm_mode`, the TMC2209's `en_spreadcycle` inverted) and `stealthChopMax` (full steps/s above which spreadCycle takes over: `TPWMTHRS` = f_CLK / (256 · v), 0 for stealthChop at any speed), and `deadman`. The chopper's `toff` (1 to 15), `hstrt`, `hend` and `tbl` are in the datasheet's units, defaults 3, 4, 1, 2. stealthChop's PWM (`PWMCONF`, the same on both chips) is `pwm`: `ofs` and `grad` (autoscale's starting amplitude, at standstill and with speed), `freq` (0 to 3), `autoscale`, `autograd`, `freewheel` (0 to 3), `reg` (1 to 15), `lim` (0 to 15), defaulting to the chip's reset value (TMC5160 0xC40C001E, TMC2209 0xC10D0024). (Fluante's old firmware started from `ofs` 255 and `grad` 14.) Everything else (coolStep, stallGuard's threshold, the short detection) keeps the sample configuration, reachable through the register list.
- **A current for a while:** `overrideCurrent(amps)` sets the run and hold currents alike, without touching the settings, until `overrideCurrent(0)`; it isn't stored or in the tree. (Fluante's old firmware settled at 30 mA for a second before turning.)
- **Ownership,** as an axis's (§4.3): `claim(owner)` marks the driver as driven by an application's part; until `release()`, the tree refuses writes to `enabled`, `velocity`, `stop` and `target` with `busy`, "driven by <owner>", and keeps them as readouts. Code isn't refused. For a part that drives the motor itself, such as Fluante's spinner, which renews a velocity every tick.
- **Current, TMC5160:** I_rms = `GLOBAL_SCALER`/256 · (CS + 1)/32 · 0.325 V / R_sense / √2. The scaler takes the run current with CS 31 (its values 1 to 31 aren't allowed, so it is 32 to 256, 256 written as 0), and `IRUN` and `IHOLD` are then rounded at that scale: 0.5 A through 0.075 Ω is scaler 42, `IRUN` 31, and 0.25 A `IHOLD` 15. The most is 3.06 A at 0.075 Ω.
- **Current, TMC2209:** I_rms = (CS + 1)/32 · V_fs / (R_sense + 20 mΩ) / √2, V_fs 0.325 V, or 0.180 V with `vsense` when the run current would take a CS under 16 (finer steps for small currents): 0.5 A through 0.11 Ω is `vsense`, `IRUN` 15, and 0.25 A `IHOLD` 7. The UART configures it: `pdn_disable`, `mstep_reg_select`, `i_scale_analog` 0 (not the board's VREF potentiometer) and `multistep_filt` are set.
- **Velocity:** `setVelocity(µsteps/s)`, signed, is a deadman like an axis's manual output (§4.3): not renewed for `deadman` (1 s), it stops, and the tree hints `ui.keepalive` 400 ms. The chips count velocities in microsteps per 2^24 clock cycles (0.715 Hz at the internal 12 MHz; `clockHz` for an external clock). The TMC5160 runs it in velocity mode (`RAMPMODE` 1 or 2, `VMAX` = |v| capped at the ramp's `maxSpeed`, accelerating at `acceleration`). The TMC2209's `VACTUAL` has no ramp, and a large jump can stall a loaded motor: with its `ramp/acceleration` (µsteps/s²) the driver ramps it every tick, up, down, through 0 when reversing, and down on `hold()` and the deadman; with 0 (the default) it jumps, written in the tick it was set. A disable stops it at once (the power stage is off). Refused unless enabled and present. `actualVelocity()` is the velocity now: the TMC5160's `VACTUAL`, the TMC2209's ramp.
- **Positioning,** TMC5160 only: `setTarget(µsteps)` writes `RAMPMODE` 0, `VMAX` at `maxSpeed` and `XTARGET`. The ramp is a trapezoid: `AMAX` = `DMAX` = `A1` = `D1` = a · 2^41 / f_CLK² (at least 1: `D1` 0 isn't allowed when positioning), `V1` 0, `VSTART` 0, `VSTOP` 10 (the datasheet's suggestion). Defaults: 51 200 µsteps/s and 51 200 µsteps/s², a turn per second of a 200-step motor at 256 microsteps. `hold()` and a disable switch to velocity mode at `VMAX` 0, which decelerates and stays. Positions count microsteps at the configured resolution (`XACTUAL`'s unit), from wherever the chip started. A TMC5160 strapped for step and direction (`SD_MODE` high, read in `IOIN`) only drives: `setTarget()` and `setVelocity()` are refused, and the steps come from elsewhere.
- **stealthChop's voltage:** at speed it runs out of supply voltage. Measured on Fluante (2026-10-19): about 110 000 µsteps/s at 256 microsteps (430 full steps/s), 0.95 A, `PWM_SCALE_SUM` and `PWM_OFS_AUTO` at 255, the most, though without a fault. `stealthChopMax` hands such speeds to spreadCycle.
- **Status:** `standstill`, `stealthChop` (running in it now), `current` (`CS_ACTUAL`), `overtemperature`, `overtemperatureWarning`, `shortToGround` and `shortToSupply` (either coil), `openLoad` (either coil, unreliable at standstill and at speed), `driverError` (`GSTAT.drv_err`), `stallGuard` (`SG_RESULT`, lower is more load: in `DRV_STATUS` on the TMC5160, read in turn on the TMC2209), `resets`, `errors` and `lastError` (above).
- **The TMC5160's SPI:** mode 3, 4 MHz by default. A datagram is 40 bits, the address (bit 7 set to write) and 32 data bits; its reply carries the status byte and the data the previous datagram asked for, so a read is two transfers. Writes aren't acknowledged.
- **The TMC2209's UART:** a read request is 4 bytes (sync 0x05, the node's address 0 to 3 from MS1 and MS2, the register, a CRC-8), its reply 8 (0x05, 0xFF, the register, 32 data bits, the CRC); a write is 8 bytes (the register | 0x80), unanswered. The CRC is the datasheet's (polynomial 7, bits taken LSB first), which agrees with TMC-API's table. Wired with TX through 1 kΩ to PDN_UART and RX straight to it, each request comes back before the reply: the driver tells the echo from the reply on each read, and consumes the echo of each write. A read waits up to 10 ms, which the UART rounds up to whole RTOS ticks plus one (§7.3): a reply takes 1.1 ms at 115200 baud, and the full wait is only spent on a chip that doesn't answer. Up to four drivers share a UART at different addresses.

### 4.8 Register maps

A Trinamic chip has tens of registers and hundreds of fields: 62 registers and 190 fields on the TMC5160, 24 and 87 on the TMC2209 (TMC-API's TMC5160 header defines 480 field macros: 256 for the bits of the microstep table, and its fields repeated for each mode). The settings (§4.7) cover the few an application tunes; the register list covers the rest, for commissioning and experiments.

- **Generated from TMC-API.** `tools/tmc_maps.py` reads a TMC-API checkout ([analogdevicesinc/TMC-API](https://github.com/analogdevicesinc/TMC-API), MIT) and writes `src/tessermotion/tmc/<chip>_map.h`, committed: a constant per register address (`tmc5160::CHOPCONF`) and per field (`tmc5160::toff`, a `TmcField`: register, shift, width, sign), and the same as tables (`kRegisters`, `kFields`, `kMap`) with each register's access, preset and fields, and the TMC-API revision. TMC-API's C code isn't used: two of its chip headers can't be included in one C++ file (each defines its own `RegisterField`), its hardware hooks are global C functions per chip family, and its register copies are sized at compile time. Its register data is what is worth having.
- **Corrections.** The generator refuses a field whose mask and shift disagree, so new errata show. TMC-API's TMC2209 has three (`OTP_READ`'s bytes, masks 0x01, 0x02, 0x04 at shifts 0, 8, 16; `OTTRIM`, mask 0x30 at shift 8), fixed from the datasheet in the generator's table. It adds the fields TMC-API doesn't give, from the datasheet: the TMC2209's `TCOOLTHRS`, `SGTHRS` and `SG_RESULT`. It also drops what isn't a field: the SPI status byte's bits given as fields of `GSTAT`, `RAMPSTAT` and `DRV_STATUS`, and second names for the same bits (the TMC5160's `tfd_all`, `tfd_3`). Mangled names get the datasheet's (`TFD_2__0_` is `tfd`, `TFD__` is `fd3`). A register split into a bit per field (the `MSLUT` words) is its own value. Field names are lower case; a name used in two registers is prefixed with its register's.
- **Access:** `read`, `write`, `readWrite` and `readClear` (flags cleared by writing 1). The TMC5160's `INP_OUT` reads `IOIN` and writes `OUTPUT`. Presets: TMC-API's sample configuration (written by a configuration, §4.7), or a write-only register's reset value (the `MSLUT` words, `PWMCONF`: shown, not written), or 0.
- **In the tree:** `registers`, a keyed list (one node at rest: its elements are built per request), each register's name, address, access, value and decoded `fields` (`toff=3 hstrt=4 ...`), with a `write` action that takes the whole value. Writing needs an admin (`writeAccess`, TesserAPI §13.1), and `OTP_PROG` is never written: it burns the chip's fuses. Register writes aren't persisted. And `map`, a file node streaming the map as JSON (a register at a time: the TMC5160's is 14 KB), for a register editor that decodes and edits fields: a TesserUI element *(later)*, built against `fixtures/tmc5160_map.json` and `fixtures/tmc2209_map.json`.
- **Cost,** computed from the tables (not yet measured, §10): about 5 KB of flash for the TMC5160's map (2.3 KB of names, 20 bytes a register, 8 a field) and 2 KB for the TMC2209's, linked with the driver, which goes through the table to read registers in turn. The driver's copy of the registers is 512 bytes of RAM.

## 5. Sequencer *(later)*

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
- **Ownership:** while a sequence drives an axis, it holds the axis's claim (§4.3), so a write to the axis's `target` through the tree is refused with `busy`. `stop` releases it.
- **Callbacks:** `onStart` and `onStop`, so an application switches its mode when a sequence starts from the interface or a timer.
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

```cpp
Adxl345 accel(bus1);   // the azimuth Mt6701's bus
Tilt tilt(accel);      // the Part: motion.add(tilt); api.mount("tilt", tilt);
```

- **`Adxl345`** (I2C, address 0x53 or 0x1D): checks DEVID 0xE5, then sets 100 Hz (BW_RATE 0x0A), full resolution ±16 g (DATA_FORMAT 0x0B) and measure mode (POWER_CTL 0x08). It reads six bytes from 0x32, at 3.9 mg/LSB. These are the heliostat's settings. Like the MT6701, it recovers the bus after 3 failed reads in a row; `begin()` never does.
- **`Tilt`** is the Part. It reads the accelerometer every `every` ticks (default 5, so 10 Hz at 50 Hz) in `sense()`, and averages the last `samples` readings (default 8, up to 32). It reports `x`, `y`, `z`, the average corrected (below), then `pitch` = atan2(−x, √(y² + z²)) and `roll` = atan2(y, z) in degrees, and `present`.
- **Absent sensors:** a sensor that fails `begin()`, or 3 reads in a row, is absent: it isn't read, and `begin()` is tried again every 2 s, starting a new average. So an accelerometer missing from a bus it shares with an encoder costs one NACK every 2 s and never resets the bus.
- **`average(n, fn)`** *(later)*: a deferred action that collects n fresh readings and replies with their mean, for calibration routines. Not built until one needs it.

**Correction.** An accelerometer has a bias and a scale error on each axis (the ADXL345's datasheet allows about ±150 mg of zero-g offset on z and ±10% of sensitivity; the second heliostat's reads |g| ≈ 8.5 m/s²). `AccelCorrection` models them as corrected_i = scale_i · (raw_i − bias_i) for x, y, z, and the tilt applies it to the average. Cross-axis terms are left out: the ADXL345's are about 1%.

- **Points** are still readings in poses spread over every direction; nothing else is needed, since a still sensor reads g = 9.80665 m/s² in any pose. `record` takes the current average, uncorrected, and refuses while it isn't full (`samples` readings since the sensor was found) or when any reading in it is more than 0.5 m/s² from the mean (the sensor moved; its noise is about 0.05 m/s²). So: set `samples` to 32, hold a pose for longer than the average (3.2 s at 10 Hz), record, repeat. Each point has a weight; weight 0 leaves it out.
- **The fit** finds the six parameters that put the corrected points on the sphere of radius g: weighted least squares on |corrected| − g, by Gauss–Newton from no correction, a step halved while it doesn't lower the cost. Six exact poses (a tumble: each axis up, then down) fit exactly.
- **Refusals** keep the correction as it was: fewer than 6 points, a point that isn't a number, a point whose magnitude isn't within 0.5 g of 1 g (not still), no convergence, or a result past |bias| 0.5 g or a scale outside 0.5 to 2. The fit also refuses points that don't determine every parameter. Each parameter's error per unit of noise in the readings, √(W·(H⁻¹)ⱼⱼ) from the normal matrix H and the total weight W, must be at most 8. That is 1.7 for a tumble, about 2.5 for poses spread evenly and 4.5 for a typical dozen placed by hand, but 16 for z when z only ever points up: its bias then can't be told from its scale. The message names the axis to point up and down. Poses from turning about one axis (an elevation axis alone) never move that axis's reading, and are refused.
- **Residuals,** computed on each read from the points and the correction as it is: the points used, and the RMS and maximum of |corrected| − g.
- An equal scale error on all three axes doesn't change pitch and roll, only |g|; a bias or unequal scales do.
- A fit joining the accelerometer with the encoders and the mount's misalignment is application geometry (§1), not here.

### 6.2 GPS

```cpp
UartPort gpsUart({.rx = 25, .tx = 33});   // UART 1, 9600 baud
NmeaGps gps(gpsUart);                     // a Part: motion.add(gps); api.mount("gps", gps);
```

- **`NmeaGps`** on a `Uart` (default 9600 baud, the heliostat's NEO module). It parses `RMC` and `GGA` sentences, any talker (`GP`, `GN`...), with checksums required and verified. Lines that fail (checksum, malformed fields, longer than 100 characters, cut short by the next `$`) are counted and dropped; other sentences are ignored. The parser is its own host-tested class (`Nmea`), replacing TinyGPSPlus and TimeLib.
- **Reports:** `fix` (`none`, `gps`, `dgps`, `pps`, `rtk`, `floatRtk`, `estimated`, `manual`, `simulation`: GGA's quality), `satellites`, `hdop`, `latitude`, `longitude` (degrees, `double`), `altitude` (m), `utc` (ISO 8601, from the latest valid RMC), and `age` (ms since the last position, −1 before one). Positions come from a GGA with a fix or a valid RMC, and are kept when the fix is lost. Without a GGA for 3 s, the fix is `none` and `satellites` 0. `gps.fix()` gives the same as a struct, from any task.
- **Time:** the latest valid RMC (status A) with a date is kept, and `takeTime(int64_t& epochMs)` hands it over once, advanced by the time since the sentence came. A void RMC's date (the receiver's clock without a fix) is never used. An application polls it outside the control task: on TesserKIT, from `dev.onTick()`, passing it to `clock.external()` (TesserKIT §5.3). A callback from the control task couldn't do that: `external()` takes the API lock, the reverse of the lock order (§3.5).
- The UART is read every tick without blocking. Parsing happens in the control task, which is cheap at 9600 baud.
- UBX configuration (rate, constellations) and PPS *(later)*.

## 7. Drivers on ESP-IDF

### 7.1 I2C

ESP-IDF refuses to link the legacy I2C driver (`driver/i2c.h`) and the new one (`driver/i2c_master.h`) in one image: it aborts at boot with "CONFLICT! driver_ng is not allowed to be used with the legacy driver". TesserMotion uses `i2c_master`. Arduino's `Wire` does too from Arduino-ESP32 3.2 on ESP-IDF 5.4 (`esp32-hal-i2c-ng.c`), which includes 3.3.7 in the pioarduino release TesserAPI pins; older Arduino-ESP32 3.x versions use the legacy driver and can't be mixed with TesserMotion's I2C.

- `I2cBus` owns an `i2c_master_bus_handle_t` (port auto-selected unless given, internal pull-ups on) and adds device handles per address on first use, up to 4. 400 kHz by default.
- Transfers use a 20 ms timeout, so a stuck bus costs at most one tick.
- `recover()` is `i2c_master_bus_reset()`: clock pulses until the device holding SDA lets go, and a controller reset. The bus keeps its speed (the heliostat dropped to 100 kHz for good).

### 7.2 LEDC

- `Pwm` allocates LEDC timers and channels from the top of the range downwards (timer 3, channels 7, 6, ...). Arduino's `analogWrite` and `ledcAttach` allocate from the bottom, so the two rarely collide. Channels can be given explicitly.
- Two motors at the same frequency share a timer.

### 7.3 UART

`UartPort` installs `uart_driver_install` with a 256-byte receive buffer on the port given (default UART 1), with no event queue. It refuses a port whose driver is already installed (Arduino's `Serial1`, if begun).

- `write()` is `uart_write_bytes()`, false without a TX pin. Without a transmit buffer it returns once the bytes are in the hardware FIFO, which holds a TMC2209 datagram.
- RX is pulled up: ESP-IDF 6.1's `uart_set_pin()` routes it through the GPIO matrix without a pull-up, and a line left floating (a GPS unplugged) reads noise. A TMC2209's single wire idles high anyway, from the TX pin.
- `read(buf, len, timeoutMs)` is `uart_read_bytes()`, which returns as soon as `len` bytes came. The timeout is rounded up to whole RTOS ticks, plus one: a wait of n ticks ends at the n-th tick interrupt, and the first may come at once. With `pdMS_TO_TICKS()` (which rounds down) a 10 ms wait was one tick at ESP-IDF's default 100 Hz, so a TMC2209 reply (1.1 ms) that straddled a tick timed out: measured on Fluante's board (2026-10-19), 59 of 300 reads back to back, and a configuration (about 20 reads) almost never completed. With two ticks, 300 of 300.

### 7.4 SPI

- `SpiBus` initializes its host (SPI2 unless given) with `spi_bus_initialize()`, DMA channel automatic, so other code may share the bus with long transfers. A host initialized already (`ESP_ERR_INVALID_STATE`: an Ethernet chip, another `SpiBus`) is shared, with a warning: its pins stand. `SpiBus(host)` shares one on purpose.
- Devices are added on first use, by chip select, with their mode and clock, up to 3 (the hardware chip selects of a host on the ESP32).
- Transfers are `spi_device_polling_transmit()`: a few bytes, done in microseconds; the driver arbitrates the bus with other devices' transfers. Arduino's `SPI` class drives the peripheral through its own HAL, not `spi_master`: give it another host than `SpiBus`'s. Arduino-ESP32 3.x's `ETH.begin()` takes a host and pins for an SPI Ethernet chip, through `spi_master`, which is how spin-seq's W5500 can share its TMC5160's bus.

## 8. Persistence

Settings are persisted with TesserAPI's `.persist()` (TesserAPI §12): an axis's `tuning`, `limits`, `sensor/config`, `motor/config`, and its correction's `harmonics`, `points` and coefficients (`cos`, `sin`, read-only, restored when loading); a tilt's `samples` and its correction's `points`, `bias` and `scale` (read-only, restored when loading); a Trinamic driver's `config`, `chopper`, `pwm` and `ramp`, but not its register writes (§4.8); later a sequencer's frames and `loop`. Status values (`position`, `state`, ...) aren't, and neither is `enabled`: axes and drivers boot disabled, and `stopAll()` disabling them is never half-saved. As TesserAPI loads leniently, a firmware with a renamed setting boots with its default. The limits and the target are in the raw frame (§4.3), so the order settings load in doesn't matter.

## 9. The trees

Paths below are relative to where the application mounts each part. `(p)` is persisted, `(w)` watched. All angles in degrees with `.unit("°")`.

### 9.1 Axis

```
azimuth/                          label from the application
  state         enum     (w)      disabled, starting, holding, moving, sensorFault
  enabled       boolean  (w)
  stop          action            stops whatever the axis does: the setpoint becomes the position
                                  (ending a slew or a jog), and the manual output 0
  jogMinus, jogPlus  action       boolean argument, ui widget hold: true while pressed (renewed
                                  within deadman), false on release; not_allowed unless running,
                                  busy while claimed
  position      number   (w)      read-only, step 0.01, ui chart
  target        number   (w)      0..360, step 0.01; not_allowed unless running,
                                  busy while claimed (§4.3)
  error         number   (w)      read-only
  output        number   (w)      -1..1, read-only, ui chart
  motor/
    output      number   (w)      -1..1; writable only while the axis is disabled (else not_allowed);
                                  ui keepalive 400 ms, against the deadman
    duty        integer  (w)      0..100 %, read-only
    config/     (p), ui collapsed, label "Motor settings"
      invert, minOutput, shape
  sensor/
    raw         number   (w)      read-only
    faults      integer  (w)      read-only: failed reads since boot
    config/     (p), ui collapsed, label "Sensor settings"
      invert, offset (-360..360)
    correction/ ui collapsed, label "Correction" (§4.2)
      harmonics integer (p)       0..6; 0: as many as the points support, up to 3
      points    list (p)          {raw, reference, weight}: raw 0..360, reference 0..360 (the
                                  position it should show), weight 0..100 (0: left out); maxSize 64
      cos, sin  arrays (p)        6 numbers each, °: a_k, b_k; read-only, written by fit and clear
      record    action            number argument, the reference: adds a point at the current
                                  reading; not_allowed before the first reading or when full
      fit       action            fits the points and applies the correction; invalid_value with
                                  the reason when it can't
      clear     action            removes the correction; the points stay
      residuals/                  of the points with the correction as it is, on each read
        points  integer  (w)
        zero, rms, max, coverage  number (w), °
  limits/       (p)               in the raw frame
    enabled, begin, end           begin and end 0..360; visibleWhen enabled
  tuning/       (p), ui collapsed
    kp, ki, kd, tolerance, maxSpeed, integralLimit, derivativeFilter, sensorTimeout, jogSpeed, deadman
```

Every value that changes without a write through the tree (the control loop, `stopAll()`, the manual output's timeout) is watched, so subscribers see it; settings, written only through the tree, aren't. Controls come first, then the readouts. `position` and `output` have `ui.widget "chart"`, which replaces Teleplot; `target` and `error` are plain readouts (a chart of a target over 0–360 is a flat line). The objects' names stay `config`, distinguished by their labels: renaming them would make stored settings load as defaults. TesserUI keeps their history, so the device stores nothing (TesserUI §7.7).

### 9.2 Sequencer *(later)*

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
tilt/                             Tilt
  x, y, z       number (w)        m/s², read-only
  pitch, roll   number (w)        °, read-only
  present       boolean (w)
  samples       integer (p)       1..32
  average       action            (later) deferred; argument n (1..256), replies {x, y, z}
  correction/   ui collapsed, label "Correction" (§6.1)
    points      list (p)          {x, y, z, weight}: m/s², uncorrected, ±160; weight 0..100
                                  (0: left out); maxSize 32
    bias        array (p)         3 numbers, m/s²; read-only, written by fit and clear
    scale       array (p)         3 numbers; read-only, written by fit and clear
    record      action            adds a point at the current average; not_allowed without a
                                  sensor, before the average is full, when it moved, or when full
    fit         action            fits the points and applies the correction; invalid_value with
                                  the reason when it can't
    clear       action            removes the correction; the points stay
    residuals/                    of the points with the correction as it is, on each read
      points    integer  (w)
      rms, max  number (w), m/s²  of |corrected| − g

gps/                              NmeaGps
  fix           enum   (w)
  satellites    integer (w)
  hdop, latitude, longitude, altitude   number (w)
  utc           string (w)
  age           integer (w)       ms; -1 before a position
  errors        integer (w)       checksum and parse failures
```

Everything but the settings (`samples`, the correction's points, bias and scale) changes without a write, so it is watched (§9.1's rule).

### 9.6 Trinamic drivers

```
stepper/                          Tmc5160 (a Tmc2209 has the same, without target, position, and
                                  with ramp/acceleration alone)
  enabled       boolean  (w)      the power stage; refused (not_allowed) while absent; enabled,
                                  velocity, stop and target refused (busy) while claimed
  velocity      number   (w)      µsteps/s, ±1 000 000; not_allowed unless enabled (and driving:
                                  not a TMC5160 strapped for step and direction); deadman,
                                  ui keepalive 400 ms
  stop          action            velocity 0, positioning ended: the motor holds where it stops
  actualVelocity number  (w)      µsteps/s, read-only, ui chart
  target        integer  (w)      µsteps; not_allowed likewise
  position      integer  (w)      µsteps, read-only, ui chart
  status/       (w)               present, standstill, stealthChop, current (0..31), overtemperature,
                                  overtemperatureWarning, shortToGround, shortToSupply, openLoad,
                                  driverError, stallGuard, resets, errors, lastError (enum)
  config/       (p), label "Driver settings"
    runCurrent, holdCurrent       A, 0..the most the sense resistor allows
    microsteps                    1..256, a power of 2 (else invalid_value)
    interpolate, invert, stealthChop
    stealthChopMax                steps/s, 0..10 000; visibleWhen stealthChop
    deadman                       ms, 500..10 000
  chopper/      (p), ui collapsed  toff 1..15, hstrt 0..7, hend 0..15, tbl 0..3
  pwm/          (p), ui collapsed, label "stealthChop PWM": ofs, grad 0..255, freq 0..3, autoscale,
                                  autograd, freewheel 0..3, reg 1..15, lim 0..15
  ramp/         (p), ui collapsed  maxSpeed (µsteps/s), acceleration (µsteps/s²; TMC2209: 0 jumps)
  registers     list              keyed by name, read-only, write access admin, ui collapsed:
                                  {name, address, access (enum), value, fields (string), write
                                  (action, the whole value; not_allowed for a read-only register)}
  map           file              the register map, JSON (§4.8)
```

Not watched: the register list (its values change every tick; a list is one value, §4.4 of TesserAPI). The settings follow §9.1's rule.

### 9.4 Motion

`describe(o, motion)` (the application mounts it where it likes, such as `/sys/motion` on TesserKIT): `period` (ms, watched), `tickMax` (µs, reset on read, so not watched), `overruns` (watched), and `stopAll` (an action with `ui.style "danger"`).

### 9.5 From the heliostat's REST routes

For the helionext port (TesserKIT §10.1), the heliostat's `/rest/heliostat/azimuth` router becomes:

| Today | TesserMotion |
|---|---|
| `position`, `target`, `tolerance`, `offset`, `invert`, `enabled` | `position`, `target`, `tuning/tolerance`, `sensor/config/offset`, `sensor/config/invert` (and the motor's `invert`, §4.3), `enabled` (not persisted) |
| `P`, `I`, `D`, `S`, `maxSpeed`, `ramp` | `tuning/kp`, `ki`, `kd`, (removed), `maxSpeed`, (removed) |
| `curGain`, `derivative`, `integral`, `encoderError` | `output`, (internal), (internal), `state` and `sensor/faults` |
| `autotune {start, cancel, amp, band, active, done, Ku, Tu}` | `autotune/` *(later)* |
| `limits {enabled, begin, end}` | `limits/`, converted to the raw frame: raw = (output − offset), negated if the encoder is inverted |
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
| + a TMC5160 and a TMC2209, with their trees | |
| RAM at rest, the task's stack included | |

## 11. Testing

1. **Host tests** (CMake, ASan and UBSan, the warnings of TesserAPI's AGENTS.md), against `test/fake_hw.*`:
   - **`tessermotion_testing`:** a CMake library with the fakes (`fake_hw.h`: GPIO, PWM, I2C register files with NACKs and a stuck bus, a settable clock, a control task run by hand) and the plant (`plant.h`), for applications' host tests too. `fake_hw.cpp` defines the platform hooks `tessermotion_core` calls, so its objects are linked into each executable that links `tessermotion_testing` rather than archived: GNU ld scans an archive once, and would pass the testing archive before the core asks for the hooks. `tessermotion_link_check`, an application naming no fake, checks this; CI builds on Linux.
   - **Plant:** a geared DC motor and its encoder in one object, a `DcMotor` and an `AngleSensor`, so `Axis axis(plant, plant)` runs the output mapping too. Speed follows the duty with a first-order lag (τ 50 ms, 30 °/s at full duty), above a dead band of 0.12 that static friction makes (below `minOutput`, as on the heliostat); duty 0 brakes. The encoder quantizes to 14 bits after ±0.01° of noise, and its reads can be made to fail.
   - **Axis:** reaches and holds targets within tolerance, at 20 and 50 Hz; never crosses the forbidden arc (randomized targets, offsets, inversion and speeds); clamps to the nearer end; slew speed respected; sensor timeout stops the motor and recovers; no jump at start; an offset or inversion change doesn't move the motor and keeps the limits at the same stops; disabled means the manual output, which drops to 0 when not rewritten for `deadman`; jog at `jogSpeed`, renewed or stopped by its deadman, clamped at the limits, ended by a target, a hold or a disable; the integral removes the error a dead band leaves.
   - **Motion:** tick order (every sense, then every update, in add order), the task and its statistics, `stopAll`, an application part setting a target inside the tick.
   - **Autotune** *(later)* finds the plant's Ku and Tu within 20 % of their analytic values, and the gains it writes hold a target.
   - **Motor:** output mapping (clamping, NaN, shape), invert, the TB6612 truth table on fake GPIOs, standby held low when the PWM can't attach.
   - **Sequencer:** before the first frame, between frames, after the end, looping, zero duration, empty tracks (NaN), wrap, unsorted writes, refusal of axis target writes while running.
   - **Correction:** known harmonics recovered from points with any zero (±180 included); apply and remove inverse to 10⁻⁴°; a short arc corrected on the arc and small off it; weights (0 leaves out, twice equals weight 2, light points move it less); automatic harmonics; refusals leave the correction unchanged; coverage across 0°. In an axis on a plant with an off-centre magnet: targets reached within 0.2° after a calibration from 24 points; a fit while holding doesn't move the motor; references through an inverted sensor with an offset, and the zero to add.
   - **Drivers:** MT6701 register traffic on the fake bus, 0° as a valid reading, recovery after 3 failures; ADXL345 register traffic and scaling, an absent ADXL345 sharing a bus with an MT6701 (never read, never recovering the bus), `Tilt`'s averaging, loss and return; NMEA sentences (any talker, both hemispheres, bad checksums, missing checksums, malformed fields, partial and over-long lines, no fix, void RMC), `NmeaGps`'s fix, staleness and `takeTime()`. The sentences are standard examples; real captures from the heliostat's module go in `test/nmea/` once taken.
   - **Accelerometer correction:** bias and scale recovered from 14 noisy poses spread evenly, and exactly from a tumble's six; refusals (too few points, weights 0 included, not a number, far from 1 g, poses turned about y alone, z only up) leave the correction unchanged, and poses down to 30° below the horizon are enough; weight 0 leaves out a moved point that otherwise pulls the fit off; in a tilt: the correction applied to the average and to pitch, points uncorrected, refused without a sensor, before the average is full and while it holds two poses.
   - **Trinamic drivers** (`test/fake_tmc.*`: a TMC5160 behind its SPI datagrams, the read pipeline included, and a TMC2209 behind its UART datagrams with CRCs, an echo or none, and addresses): the generated maps (datasheet positions, the corrected errata, tables matching constants); `TmcField` sign extension and truncation; the configuration a `begin()` writes (sample configuration, current scaling on both chips, the ramp, `GSTAT` cleared, the enable pin), then nothing while nothing changes; absent, then found after 2 s; enabling (`TOFF`, the pin); velocities (units, the cap, `RAMPMODE` 2 for negative, `VACTUAL` two's complement, the deadman renewed and expiring); positioning (`RAMPMODE`, `VMAX`, `XTARGET` in that order; hold); step and direction refusals; a reset (counted once, reconfigured with the settings, disabled, the position from the chip); power lost and back; register writes (refused for read-only registers and `OTP_PROG`, unowned bits kept, owned ones following their setting, flags cleared, kept across a reset); every register read in turn; status flags; settings into registers; `stopAll()`; CRCs against TMC-API's; addresses on a shared UART; the TMC2209's ramp (a step a tick, through 0, down on `hold()` and the deadman, a disable at once); PWM settings defaulting to the chips' reset values and owning their bits; the current override; failure reasons (an absent TMC5160, a wrong version, an echo alone, silence, a bad CRC) with searches counted; `stallGuard` and `actualVelocity`; claims refusing the tree's controls but not code.
   - **Trees:** requests through TesserAPI's handler (refusals, claims, settings, what is persisted), and golden schemas in `fixtures/*.json` (axis, motion, tilt, gps, tmc5160, tmc2209 now, with the maps `tmc5160_map.json` and `tmc2209_map.json`; sequencer later) written by `tessermotion_fixtures`, as in TesserKIT §12, to be rendered in TesserUI's golden-model tests. They replace TesserUI's hand-written `motors` fixture (a gap report to TesserUI's agent).
2. **On target:** `examples/idf/axis` (one axis on the heliostat's azimuth pins, the ADXL345 on its bus and the GPS, on TesserKIT for Wi-Fi and the interface; later a sequencer), driven by `test/e2e_motion.py` over HTTP and WebSocket: enable, move to targets and read back positions within tolerance, slew timing, limits, a sequence run, a sensor fault (unplugging is manual; the script asks), autotune, persistence across a restart. Motors move: see AGENTS.md before running it.
3. **CI:** host tests, ESP-IDF builds (5.2, 5.4, 5.5, latest × ESP32, ESP32-C3, ESP32-S3), Arduino-ESP32 3.x builds of the examples.

## 12. Packaging

```
TesserMotion/
  docs/DESIGN.md
  src/TesserMotion.h
  src/tessermotion/            drivers, controllers, *_tree.cpp
  src/tessermotion/platform/esp/
  examples/idf/axis/
  examples/arduino/Axis/, examples/arduino/Stepper/
  fixtures/
  tools/tmc_maps.py            generates src/tessermotion/tmc/ from TMC-API (§4.8)
  test/                        host tests, fake_hw, nmea captures, e2e_motion.py
  CMakeLists.txt, idf_component.yml, library.json, library.properties
```

- ESP-IDF component and PlatformIO/Arduino library, like its siblings. Namespace `tessermotion`. TesserAPI is a hard dependency, found as TesserKIT finds it: any component name ending in `tesserapi`, and `TESSERAPI_DIR` (default `../TesserAPI`) or GitHub main for the host build.
- `REQUIRES` `esp_driver_i2c` and `esp_driver_spi` (the bus headers include theirs), `PRIV_REQUIRES` `esp_driver_gpio`, `esp_driver_ledc`, `esp_driver_uart`, `esp_timer`. These are components of ESP-IDF itself, so a project that doesn't instantiate a part pays nothing for them.
- MIT, semver, public at <https://github.com/leokeba/TesserMotion> once Leo creates it.

## 13. Later

| Item | For |
|---|---|
| Steppers: `StepperAxis` (degrees, on a driver of §4.7, with an encoder for closed loop), a step generator for step and direction inputs (RMT or MCPWM; spin-seq drives its TMC5160 by step and direction, through FastAccelStepper), and an automatic calibration feeding §4.2's correction (steps as references, both directions to cancel backlash; it replaces spin-seq's 128-point linearization table) | spin-seq, TesserKIT phase 4 |
| A TesserUI register editor for §4.8's maps: fields decoded and edited from `map` (a gap report to TesserUI's agent) | commissioning Trinamic drivers |
| Multi-turn angle sensors (unwrapped position, turns) | continuous rotation axes |
| Other encoders: AS5600 (I2C), AS5048A (SPI) | other mechanics |
| A command sequencer (spin-seq's typed commands, TesserKIT §10.2) | spin-seq |
| UBX configuration and PPS for the GPS | time accuracy |
| MCPWM motor driver with PCNT encoders (`SOC_*`-guarded) | quadrature-encoded motors |
| Synchronized starts across nodes (NowTP's shared clock) | fleets |

## 14. Roadmap

| Phase | Content | Acceptance |
|---|---|---|
| 1 | Packaging, platform interfaces and fakes, the plant model, CI-ready host build | An empty `Motion` ticks on the host and builds for ESP32, C3, S3 on ESP-IDF and Arduino. Done, without CI (no GitHub repository yet); the ESP-IDF build was checked on ESP32 and S3 (no RISC-V toolchain here), Arduino on all three. |
| 2 | `DcMotor`/`Tb6612Channel`, `Mt6701`, `Axis`, `Motion`, their trees; jog with a deadman; autotune when an application asks | §11's axis tests pass (done); the heliostat's axes hold targets from the web interface (done, commissioned with tiny-heliostat). Jog done; autotune not asked for yet. |
| 3 | `Sequencer` | The heliostat's azimuth and elevation sequences run from `x-keyframes`. |
| 4 | `Adxl345`, `NmeaGps` | Tilt and GPS fix reported on the heliostat; the clock set from GPS through TesserKIT. Done on the second heliostat (2026-10-16): the MT6701 sharing its bus with the ADXL345 had no faults, GPS time reached TesserKIT's clock, ticks took at most 3.7 ms of 20. |
| 5 | Handover | TesserKIT's phase 3 (helionext) uses TesserMotion; footprint measured (§10). |
