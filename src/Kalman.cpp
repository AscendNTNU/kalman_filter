#include "kalman.h"
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Geometry>

Kalman::Kalman(){
	ros::NodeHandle n;
	
	pub_position = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/estimation/twist", 100);

	int freq;
	n.getParam("/kalman_filter/prediction_freq", freq); 
	float stepSize = 1.0/freq; 

	first_iteration = true; 

	X_priori.setZero();   
	Z_k.setZero();
	F_k.setIdentity();
	F_k(0,3) = 1.0*stepSize;
	F_k(1,4) = 1.0*stepSize; 
	F_k(2,5) = 1.0*stepSize;
	F_k_inv = F_k.inverse(); 
	B_k.setZero();  
	H_k.setZero();
	Q_k.setZero();
	R_k.setZero();  
	P_priori.setZero();
	P_posteriori.setZero(); 
	I.setIdentity();

	//Load parameters from rosparam 
	n.getParam("/kalman_filter/error_variance_x", P_posteriori(0,0)); 
	n.getParam("/kalman_filter/error_variance_y", P_posteriori(1,1)); 
	n.getParam("/kalman_filter/error_variance_z", P_posteriori(2,2)); 
	n.getParam("/kalman_filter/error_variance_x_dot", P_posteriori(3,3)); 
	n.getParam("/kalman_filter/error_variance_y_dot", P_posteriori(4,4)); 
	n.getParam("/kalman_filter/error_variance_z_dot", P_posteriori(5,5)); 

	n.getParam("/kalman_filter/prosess_variance_x", Q_k(0,0)); 
	n.getParam("/kalman_filter/prosess_variance_y", Q_k(1,1)); 
	n.getParam("/kalman_filter/prosess_variance_z", Q_k(2,2)); 
	n.getParam("/kalman_filter/prosess_variance_x_dot", Q_k(3,3)); 
	n.getParam("/kalman_filter/prosess_variance_y_dot", Q_k(4,4)); 
	n.getParam("/kalman_filter/prosess_variance_z_dot", Q_k(5,5)); 
	//Variance of the measurements
	n.getParam("/kalman_filter/observation_variance_x", R_k(0,0)); 
	n.getParam("/kalman_filter/observation_variance_y", R_k(1,1)); 
	n.getParam("/kalman_filter/observation_variance_z", R_k(2,2)); 
	//n.getParam("/kalman_filter/observation_variance_x_dot", R_k(3,3)); 
	//n.getParam("/kalman_filter/observation_variance_y_dot", R_k(4,4)); 
	//n.getParam("/kalman_filter/observation_variance_z_dot", R_k(5,5)); 
	//Setting the inital state of the system

	n.getParam("/kalman_filter/init_x", X_posteriori(0)); 
	n.getParam("/kalman_filter/init_y", X_posteriori(1)); 
	n.getParam("/kalman_filter/init_z", X_posteriori(2)); 
	n.getParam("/kalman_filter/init_x_dot", X_posteriori(3)); 
	n.getParam("/kalman_filter/init_y_dot", X_posteriori(4)); 
	n.getParam("/kalman_filter/init_z_dot", X_posteriori(5)); 
	//Setting which states that are measured
	n.getParam("/kalman_filter/observation_x", H_k(0,0));
	n.getParam("/kalman_filter/observation_y", H_k(1,1));
	n.getParam("/kalman_filter/observation_z", H_k(2,2));
	//n.getParam("/kalman_filter/observation_x_vel", H_k(3,3));
	//n.getParam("/kalman_filter/observation_x_vel", H_k(4,4));
	//n.getParam("/kalman_filter/observation_x_vel", H_k(5,5));
};


void Kalman::printSystem(){
	std::cout << "F_k" << std::endl; 
	std::cout << F_k << std::endl;
	std::cout << "F_k_inverse" << std::endl; 
	std::cout << F_k_inv << std::endl; 
	std::cout << "B_k" << std::endl; 
	std::cout << B_k << std::endl; 
	std::cout << "H_k" << std::endl; 
	std::cout << H_k << std::endl; 
	std::cout << "P_posteriori" << std::endl; 
	std::cout << P_posteriori << std::endl; 
	std::cout << " Q_k " << std::endl; 
	std::cout << Q_k << std::endl; 
	std::cout << "R_k" << std::endl; 
	std::cout << R_k << std::endl; 
	std::cout << "Initial states" << std::endl; 
	std::cout << X_posteriori << std::endl; 
};


void Kalman::prediction(){
	X_priori = F_k*X_posteriori; //+ B_k U 
	P_priori = F_k*P_posteriori*F_k.transpose() + Q_k; 

	P_posteriori = P_priori; // This is nescessary if the positon data is stopped for whatever reason
}


void Kalman::correction(const geometry_msgs::PoseStamped& input){
	
	Z_k << input.pose.position.x, input.pose.position.y, input.pose.position.z; 
	
	Y_k = Z_k - H_k*X_priori; 
	S_k = H_k*P_priori*H_k.transpose() + R_k; 
	K_k = P_priori*H_k.transpose()*S_k.inverse();
	X_posteriori = X_priori + K_k*Y_k; 
	P_posteriori = (I - K_k*H_k)*P_priori;
}


void Kalman::publish(){
	
	pose_estimate.pose.position.x = X_posteriori(0);
	pose_estimate.pose.position.y = X_posteriori(1);
	pose_estimate.pose.position.y = X_posteriori(2);
	pose_estimate.header.stamp = ros::Time::now(); 
	
	twist_estimate.twist.linear.x = X_posteriori(3);
	twist_estimate.twist.linear.y = X_posteriori(4);
	twist_estimate.twist.linear.z = X_posteriori(5); 
	twist_estimate.header.stamp = ros::Time::now();

	pub_position.publish(pose_estimate);
	//pub_velocity.publish(twist_estimate); 
}


void Kalman::updateOrientation(const geometry_msgs::PoseStamped& input){
	auto quat = input.pose.orientation;
	Eigen::Quaternionf q(quat.w, quat.x, quat.y, quat.z); 
    q.normalize(); 
    rotation_matrix = q.toRotationMatrix(); 
    orientation_rpy = rotation_matrix.eulerAngles(0,1,2);
}










/*
//The time jumb is based on the time delay of the system 
void Kalman::rollBack(int num_of_itr){
	//Roll back a given number of steps based on the model of the system! 
	for (int i = 0; i < num_of_itr; i++){
		X_priori = F_k_inv*X_priori;	//-F_k_inv*b_k u  
		P_priori = F_k_inv*(P_priori - Q_k)*F_k_inv.transpose(); 
	}
}

void Kalman::rollForward(){

}*/