#include "tessermotion/trees.h"

namespace tessermotion {

void describe(tesser::Object& o, NmeaGps& g) {
    o.value("fix", [&g] { return g.fix().quality; },
            {"none", "gps", "dgps", "pps", "rtk", "floatRtk", "estimated", "manual", "simulation"})
        .watch();
    o.value("satellites", [&g] { return g.fix().satellites; }).watch();
    o.value("hdop", [&g] { return g.fix().hdop; }).step(0.01).watch().doc("Horizontal dilution of precision");
    o.value("latitude", [&g] { return g.fix().latitude; }).unit("°").step(0.000001).watch();
    o.value("longitude", [&g] { return g.fix().longitude; }).unit("°").step(0.000001).watch();
    o.value("altitude", [&g] { return g.fix().altitude; }).unit("m").step(0.1).watch();
    o.value("utc", [&g] { return g.utc(); }).watch().doc("The receiver's time, from its latest valid RMC");
    o.value("age", [&g] { return g.fix().ageMs; }).unit("ms").watch().doc("Since the last position; -1: none yet");
    o.value("errors", [&g] { return g.errors(); }).watch().doc("Sentences dropped: checksums, malformed or cut short");
}

}  // namespace tessermotion
