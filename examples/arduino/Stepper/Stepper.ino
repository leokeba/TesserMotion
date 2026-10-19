// Two Trinamic stepper drivers on Arduino: a TMC5160 on SPI at /stepper,
// with its ramp generator, and a TMC2209 on its UART at /small, with the
// control loop's statistics at /sys/motion. TesserKIT gives it Wi-Fi and the
// web interface: flash it, join the "stepper-xxxx" network from a phone, and
// the interface opens.
//
// The drivers boot disabled. Set the motor's current (config/runCurrent)
// before enabling one; registers/ lists every register, for an admin to
// write. Pins: a classic ESP32 DevKitC.
#include <TesserKIT.h>
#include <TesserMotion.h>

#if __has_include("secrets.h")
#include "secrets.h"  // gitignored: WIFI_SSID, WIFI_PASSWORD for development
#endif

using namespace tessermotion;

tesserkit::Device dev("Stepper");
tesserkit::System sys(dev);
tesserkit::Network net(dev);
tesserkit::Firmware firmware(dev);

SpiBus spi({.sclk = 18, .mosi = 23, .miso = 19});  // SPI2
Tmc5160 stepper(spi, {.cs = 5, .enable = 4});      // 0.075 Ω sense resistors
// TX through 1 kΩ to PDN_UART, RX straight to it. UART 1, 115200 baud.
UartPort tmcUart({.rx = 16, .tx = 17, .baud = 115200});
Tmc2209 small(tmcUart, {.address = 0, .enable = 21});  // 0.11 Ω
Motion motion;

void setup() {
    Serial.begin(115200);
#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
    net.defaultNetwork(WIFI_SSID, WIFI_PASSWORD);
#endif
    stepper.ramp.maxSpeed = 25600;  // µsteps/s, until the stored settings say otherwise
    motion.add(stepper);
    motion.add(small);
    dev.api().mount("stepper", stepper).label("TMC5160");
    dev.api().mount("small", small).label("TMC2209");
    dev.api().object("sys").mount("motion", motion).label("Motion");
    dev.onState([](tesserkit::State s) {
        if (s == tesserkit::State::Updating) motion.stopAll();  // nothing moves during an update
    });
    dev.begin();
    motion.start(50);
}

void loop() { delay(1000); }
