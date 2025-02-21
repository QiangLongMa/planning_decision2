#include "runsocketcan.h"

runsocketcan::runsocketcan() {}

void runsocketcan::run() {
    system(" ros2 run can can_node ");
}
