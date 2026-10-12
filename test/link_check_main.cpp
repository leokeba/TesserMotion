// An application's host build: it links tessermotion_testing but names no
// fake, so only tessermotion_core asks for the platform hooks. Building it
// is the check (GNU ld; ld64 rescans archives and would pass anyway).

#include "tessermotion/motion.h"
#include "tessermotion/tb6612.h"

int main() {
    tessermotion::Tb6612Standby standby(18);
    tessermotion::Tb6612Channel motor(standby, {16, 17, 5});
    tessermotion::Motion motion;
    motor.begin();
    return motion.start(50) ? 0 : 1;
}
