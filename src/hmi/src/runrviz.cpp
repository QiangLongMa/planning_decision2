#include "runrviz.h"

runrviz::runrviz() {}

void runrviz::run() {
    int ros2_result = system("ros2 run myrviz points_and_lines_publisher");
    if (ros2_result != 0) {
        std::cerr << "Failed to run ros2 command\n";
        // Handle error or exit
        return;
    }

    int rviz2_result = system("rviz2 /home/mm/longshan6.25/src/myrviz/default.rviz");
    if (rviz2_result != 0) {
        std::cerr << "Failed to run rviz2 command\n";
        // Handle error or exit
        return;
    }
    //system("ros2 run myrviz points_and_lines_publisher && rviz2 /home/mm/longshan6.25/src/myrviz/default.rviz");
    //system("rviz2 /home/mm/longshan6.25/src/myrviz/default.rviz");
}
