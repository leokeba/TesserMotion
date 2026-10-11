// Host test runner: ./tessermotion_tests [filter]
#include "driver_cases.h"
#include "axis_cases.h"
#include "motion_cases.h"
#include "tree_cases.h"

int main(int argc, char** argv) { return check::run(argc > 1 ? argv[1] : nullptr) ? 1 : 0; }
