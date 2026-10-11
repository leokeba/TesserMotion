// One closed-loop axis on Arduino: a TB6612 channel and an MT6701 on I2C, at
// /azimuth, with the control loop's statistics at /sys/motion. TesserKIT
// gives it Wi-Fi and the web interface: flash it, join the "axis-xxxx"
// network from a phone, and the interface opens.
//
// The axis boots disabled. Before enabling it on real mechanics, set limits
// and a low maxSpeed. Pins: the heliostat's DevKitC.
#include <TesserKIT.h>
#include <TesserMotion.h>

#if __has_include("secrets.h")
#include "secrets.h"  // gitignored: WIFI_SSID, WIFI_PASSWORD for development
#endif

using namespace tessermotion;

tesserkit::Device dev("Axis");
tesserkit::System sys(dev);
tesserkit::Network net(dev);
tesserkit::Firmware firmware(dev);

Tb6612Standby bridge(18);
Tb6612Channel motor(bridge, {.pwm = 16, .in1 = 17, .in2 = 5});
I2cBus bus({.sda = 13, .scl = 14});
Mt6701 encoder(bus);
Axis azimuth(encoder, motor);
Motion motion;

void setup() {
    Serial.begin(115200);
#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
    net.defaultNetwork(WIFI_SSID, WIFI_PASSWORD);
#endif
    azimuth.tuning.maxSpeed = 10;  // °/s, until the stored settings say otherwise
    motion.add(azimuth);
    dev.api().mount("azimuth", azimuth).label("Azimuth");
    dev.api().object("sys").mount("motion", motion).label("Motion");
    dev.onState([](tesserkit::State s) {
        if (s == tesserkit::State::Updating) motion.stopAll();  // nothing moves during an update
    });
    dev.begin();
    motion.start(50);
}

void loop() { delay(1000); }
