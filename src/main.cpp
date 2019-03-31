#include <ros/ros.h>
#include "kalman.h"
#include <iostream>
#include <tf/tf.h>
#include <Eigen/Geometry>

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
    ros::Subscriber sub_orientation = n.subscribe("/mavros/local_position/pose",1, &Kalman::updateOrientation, &filter);
 
    Eigen::Quaternionf q(0.7071068, 0, 0, 0.7071068); 
    q.normalize(); 
    auto rotation_matrix = q.toRotationMatrix(); 
    //orientation_rpy = rotation_matrix.eulerAngles(0,1,2); 
    Matrix<float, 3, 1> test;
    test << 1, 0, 0; 

    std::cout << "Test: " << rotation_matrix*test << std::endl; 
    while(ros::ok()){
        filter.prediction(); 
        ros::spinOnce();
        filter.publish(); 
        rate.sleep();
    }
}
