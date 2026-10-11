// One closed-loop axis on the heliostat's azimuth (docs/DESIGN.md section
// 11): a TB6612 channel and an MT6701 on I2C, at /azimuth, with the control
// loop's statistics at /sys/motion. TesserKIT gives it Wi-Fi and the web
// interface: flash it, join the "tessermotion-axis-xxxx" network from a phone,
// and the interface opens.
//
// The axis boots disabled. Before enabling it on real mechanics, set limits
// and a low maxSpeed (see AGENTS.md).
//
// For development, a gitignored main/secrets.h may define WIFI_SSID and
// WIFI_PASSWORD, and WIFI_TX_POWER (dBm) for boards with a weak supply.
#include "TesserKIT.h"
#include "TesserMotion.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

using namespace tessermotion;

namespace {

tesserkit::Device dev("TesserMotion Axis");
tesserkit::System sys(dev);
tesserkit::Network net(dev);
tesserkit::Firmware firmware(dev);

// The heliostat's DevKitC pins.
Tb6612Standby bridge(18);
Tb6612Channel motor(bridge, {.pwm = 16, .in1 = 17, .in2 = 5});
I2cBus bus({.sda = 13, .scl = 14});
Mt6701 encoder(bus);
Axis azimuth(encoder, motor);
Motion motion;

}  // namespace

extern "C" void app_main() {
#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
    net.defaultNetwork(WIFI_SSID, WIFI_PASSWORD);
#endif
#if defined(WIFI_TX_POWER)
    net.txPower(WIFI_TX_POWER);
#endif
    azimuth.tuning.maxSpeed = 10;  // °/s, until the stored settings say otherwise
    motion.add(azimuth);
    dev.api().mount("azimuth", azimuth).label("Azimuth");
    dev.api().object("sys").mount("motion", motion).label("Motion");
    // Nothing moves while new firmware is written.
    dev.onState([](tesserkit::State s) {
        if (s == tesserkit::State::Updating) motion.stopAll();
    });
    dev.begin();  // loads the axis's stored settings
    motion.start(50);
}
