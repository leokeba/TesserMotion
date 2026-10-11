#include "tessermotion/trees.h"

namespace tessermotion {

void describe(tesser::Object& o, Motion& m) {
    o.value("period", [&m] { return m.periodMs(); }).unit("ms").watch();
    o.value("tickMax", [&m] { return m.tickMaxUs(); }).unit("µs").doc("The longest tick since the last read");
    o.value("overruns", [&m] { return m.overruns(); }).doc("Ticks longer than the period");
    o.action("stopAll", [&m] { m.stopAll(); }).doc("Disables every axis").ui("style", "danger").ui("icon", "stop");
}

}  // namespace tessermotion
