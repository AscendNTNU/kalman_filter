#include <ros/ros.h>
#include "kalman.h"
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;

    int frequency; 
    n.getParam("/kalman_filter/prediction_freq", frequency); 
    ros::Rate rate(30); 
    
    std::cout << "Freq: " << frequency << " Hz" << std::endl; 
    Kalman filter; 

    filter.printVector(); 

    ros::Subscriber sub = n.subscribe("/measurement/pose", 1, &Kalman::correction, &filter); 

    
    while(ros::ok()){
        filter.prediction(); 
        ros::spinOnce();
        filter.publish(); 
        rate.sleep();
    }
}
