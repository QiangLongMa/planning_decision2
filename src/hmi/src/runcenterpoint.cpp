#include "runcenterpoint.h"

runcenterpoint::runcenterpoint(){}

void runcenterpoint::run(){
    system("ros2 run lidar_processing Lidar_Processing_node");

}