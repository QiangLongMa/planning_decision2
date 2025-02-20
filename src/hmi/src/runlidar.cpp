#include "runlidar.h"

runlidar::runlidar(){}

void runlidar::run(){
    //system("ros2 run centerpoint centerpoint_node");
    //system("ros2 run lidar_rec lidar_rec_node");
    system("ros2 run dipgseg demo_node");

}