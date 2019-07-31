#include <ros/ros.h>
#include "kalman.h"
#include <iostream>
#include <tf/tf.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pub; 
void republish(const geometry_msgs::PoseWithCovarianceStamped& input){
	pub.publish(input);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;

    int frequency; 
    n.getParam("kalman_filter/prediction_freq", frequency); 
    ros::Rate rate(30); 
    
    Kalman filter; 

    filter.printSystem(); 

    ros::Subscriber sub_pose = n.subscribe("zed/zed_node/pose_with_covariance", 1, republish); 
	pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("mavros/vision_pose/pose_cov", 1);
	ros::spin();

    //while(ros::ok()){
    //    filter.prediction(); 
    //    ros::spinOnce();
    //    filter.publish(); 
    //    rate.sleep();
    //}
}
