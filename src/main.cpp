#include <ros/ros.h>
#include "kalman.h"
#include <iostream>
#include <tf/tf.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;

    int frequency; 
    n.getParam("/kalman_filter/prediction_freq", frequency); 
    ros::Rate rate(30); 
    
    std::cout << "Freq: " << frequency << " Hz" << std::endl; 
    Kalman filter; 

    filter.printSystem(); 

    ros::Subscriber sub_velocity = n.subscribe("/measurement/twist", 1, &Kalman::correction, &filter); 
    ros::Subscriber sub_orientation = n.subscribe("/mavros/local_position/pose",1, &Kalman::updateQuat, &filter);
   

    while(ros::ok()){
        filter.prediction(); 
        ros::spinOnce();
        filter.publish(); 
        rate.sleep();
    }
}
