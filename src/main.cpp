#include <ros/ros.h>
#include "kalman.h"
#include <iostream>
#include <tf/tf.h>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;

    int frequency; 
    n.getParam("kalman_filter/prediction_freq", frequency); 
    ros::Rate rate(30); 
    
    Kalman filter; 

    filter.printSystem(); 

    ros::Subscriber sub_pose = n.subscribe("mavros/mocap/pose", 1, &Kalman::correction, &filter); 
    while(ros::ok()){
        filter.prediction(); 
        ros::spinOnce();
        filter.publish(); 
        rate.sleep();
    }
}
