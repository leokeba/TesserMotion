# TesserMotion design

Status: phase 1 and the core of phase 2 implemented (2026-10-11): the platform layer, `DcMotor`/`Tb6612Channel`, `Mt6701`, `Axis`, `Motion` and their trees, host-tested against a simulated plant, not yet run on hardware. This is the specification the code follows; when the two disagree, fix one of them. Sections marked *(later)* are committed to but not implemented yet. References to TesserAPI, TesserUI and TesserKIT point to their own DESIGN.md: [TesserAPI](https://github.com/leokeba/TesserAPI/blob/main/docs/DESIGN.md), [TesserUI](https://github.com/leokeba/TesserUI/blob/main/docs/DESIGN.md), [TesserKIT](https://github.com/leokeba/TesserKIT/blob/main/docs/DESIGN.md).

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
- Steppers in the first release (§13).
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
                   └─▶ Accelerometer (Adxl345 on I2cBus), Gps (Nmea on Uart)

 tesser::Api ◀── describe(Object&, Axis&), describe(Object&, Sequencer&), ...
```

### 3.1 Layers

| Layer | Content | Depends on |
|---|---|---|
| Driver | `Tb6612Channel`, `Tb6612Standby`, `Mt6701`, `Adxl345`, `NmeaGps`: register maps, pin logic, scaling. | The platform interfaces (§3.3). |
| Controller | `DcMotor` output mapping, `Axis` (with its PID), `RelayAutotune`, `Sequencer`, `Motion`. Pure logic; time comes in as an argument. | Drivers through their interfaces (`AngleSensor`, `DcMotor`, `Accelerometer`). |
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
- **`stopAll()`** disables every axis (and later stops every sequencer), from any task. An application calls it before a firmware update (TesserKIT's `dev.onState()`, state `Updating`) and from its own emergency paths.

### 3.3 Platform layer

Narrow interfaces in `src/tessermotion/platform.h`, an ESP implementation in `src/tessermotion/platform/esp/`, and fakes in `test/fake_hw.*`. A part takes the interfaces it needs, so an interface without a user isn't linked.

| Interface | Methods | ESP implementation |
|---|---|---|
| `Gpio` | `output(pin, level)`, `set(pin, level)` | `driver/gpio.h` |
| `Pwm` | `attach(pin, frequencyHz) → channel`, `duty(channel, fraction 0..1)` | LEDC (§7.2) |
| `I2c` | `begin()` (idempotent: every device on the bus calls it), `read(addr, reg, buf, len)`, `write(addr, reg, buf, len)`, `recover()` | `I2cBus`, on `driver/i2c_master.h` (§7.1) |
| `Uart` *(later)* | `begin(rx, tx, baud)`, non-blocking `read(buf, max) → n` | `driver/uart.h` |
| `Lock` | `lock()`, `unlock()`; recursive | a static FreeRTOS recursive mutex; `std::recursive_mutex` on the host |
| `startPeriodic(task, periodUs, tick)`, `nowUs()` | free functions: the control task and the clock | `xTaskCreatePinnedToCore` with `vTaskDelayUntil`; `esp_timer_get_time()` |

- **Defaults:** a TB6612 takes a `Gpio*` and a `Pwm*`, null by default, meaning the chip's (`defaultGpio()`, `defaultPwm()`). Tests pass fakes, or use the defaults, which the fakes provide on the host.
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

## 4. DC motors and axes

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

**Bus faults:** after 3 failed reads in a row (a NACK, a bus error or a 20 ms timeout, §7.1), the sensor calls `bus.recover()` and counts again from zero, so a dead bus is recovered every third tick at most. The axis sees `read()` return false and counts it (§4.3, §9.1). The heliostat's separate 100 ms cooldown and slow-transfer trigger are gone: the count spaces recoveries, and a slow transfer is a timeout.

### 4.3 Axis

An `Axis` is a closed-loop position servo: an `AngleSensor`, a `DcMotor`, and a PID between them, in degrees on a circle.

- **Two frames.** The raw frame is the sensor's reading. The output frame is what the tree and the API show and take: position = (−raw if `sensor.invert`, else raw) + `sensor.offset`, modulo 360. Internally, the target, the setpoint and the limits are all in the raw frame. So changing the offset or the sensor's `invert` (a calibration) never moves the motor, and loading persisted settings gives the same result in any order. The heliostat ran its PID in the output frame, so a new offset moved the axis and the encoder's `invert` changed the control's sign.
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
- **Disabled:** the motor gets the manual output (`motor/output`, 0 by default: a brake), and the target follows the position. Enabling or disabling resets the manual output to 0. Target writes are refused while the axis isn't running (`disabled` or `starting`), so a target set before enabling can't make it jump; manual output is refused while it's enabled.
- **Ownership:** `claim(owner)` marks the target as driven by something (a sequence, a tracker); until `release()`, tree writes to `target` are refused with `busy`, "driven by <owner>". Code isn't refused: the application decides.

**States** (an enum, read-only, watched): `disabled`, `starting` (waiting for a first reading), `holding` (at the target, within tolerance), `moving`, `sensorFault`.

**Programmatic API** (for applications such as the sun tracker): `setTarget(deg)` (through the slew), `setTargetNow(deg)` (the setpoint jumps; sequencers and trackers that already move smoothly), `hold()`, `target()`, `position()`, `error()`, `output()`, `raw()`, `faults()`, `enable(bool)`, `enabled()`, `state()`, `setManualOutput(u)`, `claim(owner)`, `release()`, `owner()`. They take Motion's lock and may be called from any task, a part's `update()` included. The settings are public fields (`tuning`, `sensorConfig`, `limits`; the motor's `invert`, `minOutput`, `shape`): set them before `Motion::start()`, or under `axis.lock()` afterwards, as the tree does.

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

## 6. Sensors *(later)*

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
- **Time:** the latest valid RMC with a date is kept, and `takeTime(int64_t& epochMs)` hands it over once. An application polls it outside the control task: on TesserKIT, from `dev.onTick()`, passing it to `clock.external()` (TesserKIT §5.3). A callback from the control task couldn't do that: `external()` takes the API lock, the reverse of the lock order (§3.5).
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

### 7.3 UART *(later)*

`Uart` installs `uart_driver_install` with a 256-byte receive buffer on the port given (default UART 1), with no event queue.

## 8. Persistence

Settings are persisted with TesserAPI's `.persist()` (TesserAPI §12): an axis's `tuning`, `limits`, `sensor/config` and `motor/config`; later a sequencer's frames and `loop`, an accelerometer's `samples`. Status values (`position`, `state`, ...) aren't, and neither is `enabled`: axes boot disabled, and `stopAll()` disabling them is never half-saved. As TesserAPI loads leniently, a firmware with a renamed setting boots with its default. The limits and the target are in the raw frame (§4.3), so the order settings load in doesn't matter.

## 9. The trees

Paths below are relative to where the application mounts each part. `(p)` is persisted, `(w)` watched. All angles in degrees with `.unit("°")`.

### 9.1 Axis

```
azimuth/                          label from the application
  position      number   (w)      read-only, step 0.01, ui chart
  target        number   (w)      0..360, step 0.01, ui chart; not_allowed unless running,
                                  busy while claimed (§4.3)
  error         number   (w)      read-only
  output        number   (w)      -1..1, read-only, ui chart
  state         enum     (w)      disabled, starting, holding, moving, sensorFault
  enabled       boolean
  stop          action            the setpoint becomes the position: stops a slew, holds
  motor/
    output      number            -1..1; writable only while the axis is disabled (else not_allowed)
    duty        integer  (w)      0..100 %, read-only
    config/     (p), ui collapsed
      invert, minOutput, shape
  sensor/
    raw         number   (w)      read-only
    faults      integer           read-only: failed reads since boot
    config/     (p), ui collapsed
      invert, offset (-360..360)
  limits/       (p)               in the raw frame
    enabled, begin, end           begin and end 0..360; visibleWhen enabled
  tuning/       (p), ui collapsed
    kp, ki, kd, tolerance, maxSpeed, integralLimit, derivativeFilter, sensorTimeout
```

`position`, `target` and `output` have `ui.widget "chart"`, which replaces Teleplot. TesserUI keeps their history, so the device stores nothing (TesserUI §7.7).

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

### 9.3 Sensors *(later)*

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

`describe(o, motion)` (the application mounts it where it likes, such as `/sys/motion` on TesserKIT): `period` (ms, watched), `tickMax` (µs, reset on read), `overruns`, and `stopAll` (an action with `ui.style "danger"`).

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
| RAM at rest, the task's stack included | |

## 11. Testing

1. **Host tests** (CMake, ASan and UBSan, the warnings of TesserAPI's AGENTS.md), against `test/fake_hw.*`:
   - **`tessermotion_testing`:** a CMake library with the fakes (`fake_hw.h`: GPIO, PWM, I2C register files with NACKs and a stuck bus, a settable clock, a control task run by hand) and the plant (`plant.h`), for applications' host tests too.
   - **Plant:** a geared DC motor and its encoder in one object, a `DcMotor` and an `AngleSensor`, so `Axis axis(plant, plant)` runs the output mapping too. Speed follows the duty with a first-order lag (τ 50 ms, 30 °/s at full duty), above a dead band of 0.12 that static friction makes (below `minOutput`, as on the heliostat); duty 0 brakes. The encoder quantizes to 14 bits after ±0.01° of noise, and its reads can be made to fail.
   - **Axis:** reaches and holds targets within tolerance, at 20 and 50 Hz; never crosses the forbidden arc (randomized targets, offsets, inversion and speeds); clamps to the nearer end; slew speed respected; sensor timeout stops the motor and recovers; no jump at start; an offset or inversion change doesn't move the motor and keeps the limits at the same stops; disabled means the manual output; the integral removes the error a dead band leaves.
   - **Motion:** tick order (every sense, then every update, in add order), the task and its statistics, `stopAll`, an application part setting a target inside the tick.
   - **Autotune** *(later)* finds the plant's Ku and Tu within 20 % of their analytic values, and the gains it writes hold a target.
   - **Motor:** output mapping (clamping, NaN, shape), invert, the TB6612 truth table on fake GPIOs, standby held low when the PWM can't attach.
   - **Sequencer:** before the first frame, between frames, after the end, looping, zero duration, empty tracks (NaN), wrap, unsorted writes, refusal of axis target writes while running.
   - **Drivers:** MT6701 register traffic on the fake bus, 0° as a valid reading, recovery after 3 failures; later ADXL345 traffic and NMEA sentences (real captures from the heliostat's module in `test/nmea/`, bad checksums, partial lines, no fix).
   - **Trees:** requests through TesserAPI's handler (refusals, claims, settings, what is persisted), and golden schemas in `fixtures/*.json` (axis and motion now; sequencer, tilt, gps later) written by `tessermotion_fixtures`, as in TesserKIT §12, to be rendered in TesserUI's golden-model tests. They replace TesserUI's hand-written `motors` fixture (a gap report to TesserUI's agent).
2. **On target:** `examples/idf/axis` (one axis on the heliostat's azimuth pins, on TesserKIT for Wi-Fi and the interface; later a sequencer and the accelerometer), driven by `test/e2e_motion.py` over HTTP and WebSocket: enable, move to targets and read back positions within tolerance, slew timing, limits, a sequence run, a sensor fault (unplugging is manual; the script asks), autotune, persistence across a restart. Motors move: see AGENTS.md before running it.
3. **CI:** host tests, ESP-IDF builds (5.2, 5.4, 5.5, latest × ESP32, ESP32-C3, ESP32-S3), Arduino-ESP32 3.x builds of the examples.

## 12. Packaging

```
TesserMotion/
  docs/DESIGN.md
  src/TesserMotion.h
  src/tessermotion/            drivers, controllers, *_tree.cpp
  src/tessermotion/platform/esp/
  examples/idf/axis/
  examples/arduino/Axis/
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
| 1 | Packaging, platform interfaces and fakes, the plant model, CI-ready host build | An empty `Motion` ticks on the host and builds for ESP32, C3, S3 on ESP-IDF and Arduino. Done, without CI (no GitHub repository yet); the ESP-IDF build was checked on ESP32 and S3 (no RISC-V toolchain here), Arduino on all three. |
| 2 | `DcMotor`/`Tb6612Channel`, `Mt6701`, `Axis`, `Motion`, their trees; jog with a deadman; autotune when an application asks | §11's axis tests pass (done); one axis of the heliostat holds targets from the web interface (not yet run on hardware). |
| 3 | `Sequencer` | The heliostat's azimuth and elevation sequences run from `x-keyframes`. |
| 4 | `Adxl345`, `NmeaGps` | Tilt and GPS fix reported on the heliostat; the clock set from GPS through TesserKIT. |
| 5 | Handover | TesserKIT's phase 3 (helionext) uses TesserMotion; footprint measured (§10). |
