#include "tessermotion/trees.h"

namespace tessermotion {

void describe(tesser::Object& o, Tilt& t) {
    o.value("x", [&t] { return t.reading().x; }).unit("m/s²").step(0.01).watch();
    o.value("y", [&t] { return t.reading().y; }).unit("m/s²").step(0.01).watch();
    o.value("z", [&t] { return t.reading().z; }).unit("m/s²").step(0.01).watch();
    o.value("pitch", [&t] { return t.reading().pitch; }).unit("°").step(0.01).watch();
    o.value("roll", [&t] { return t.reading().roll; }).unit("°").step(0.01).watch();
    o.value("present", [&t] { return t.present(); }).watch().doc("The sensor answers");
    o.value(
         "samples",
         [&t] {
             Guard g(t.lock());
             return t.samples;
         },
         [&t](uint32_t n) {
             Guard g(t.lock());
             t.samples = n;
         })
        .range(1, Tilt::kMaxSamples)
        .persist()
        .doc("Readings averaged");
}

}  // namespace tessermotion
