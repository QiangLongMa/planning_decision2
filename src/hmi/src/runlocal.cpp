#include "runlocal.h"

runlocal::runlocal() {}

void runlocal::run() {
    // std::cout << "ros2 run local  local_node" << std::endl;
    system(" ros2 run local local_node4 ");
}
