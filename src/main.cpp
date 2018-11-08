#include <ros/ros.h>
#include "kalman.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_name");

    Kalman filter; 

    filter.printVector(); 

    ros::spin();
}
