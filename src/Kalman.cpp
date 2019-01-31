#include "kalman.h"
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
Kalman::Kalman(){
	//Init vectors and matrices
	ros::NodeHandle n;
	
	pub = n.advertise<geometry_msgs::PoseStamped>("/estimation/pose", 100);

	int freq;
	n.getParam("/kalman_filter/prediction_freq", freq); 
	float stepSize = 1.0/freq; 

	first_iteration = true; 

	X_priori.setZero();   
	Z_k.setZero();
	Z_last.setZero(); //?
	F_k.setIdentity();
	F_k(0,3) = 1.0*stepSize;
	F_k(1,4) = 1.0*stepSize; 
	F_k(2,5) = 1.0*stepSize; 
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
	n.getParam("/kalman_filter/observation_variance_x_dot", R_k(3,3)); 
	n.getParam("/kalman_filter/observation_variance_y_dot", R_k(4,4)); 
	n.getParam("/kalman_filter/observation_variance_z_dot", R_k(5,5)); 
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

	n.getParam("/kalman_filter/observation_x_vel", H_k(3,3)); 
};


void Kalman::printSystem(){	

	std::cout << "F_k" << std::endl; 
	std::cout << F_k << std::endl; 
	std::cout << "B_k" << std::endl; 
	std::cout << B_k << std::endl; 
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
	X_priori = F_k*X_posteriori; // + B_k U 
	P_priori = F_k*P_posteriori*F_k.transpose() + Q_k; 
}

void Kalman::correction(const geometry_msgs::PoseStamped& input){
	
	Z_k << input.pose.position.x, input.pose.position.y, input.pose.position.z, 0, 0 ,0; //Put the measurement into a vector
	
	if(first_iteration){
		Z_last = Z_k;
		last_measurement_time = input.header.stamp; 
		first_iteration = false; 
		return; 
	}

	ros::Duration time_step = input.header.stamp - last_measurement_time;

	Z_k(3) = (Z_k(0) - Z_last(0))/time_step.toSec(); //Velocity x estimate from last measurement || Remember it is scaled with the timestep! 
	Z_k(4) = (Z_k(1) - Z_last(1))/time_step.toSec();
	Z_k(5) = (Z_k(2) - Z_last(2))/time_step.toSec();

	std::cout << "The velocities: \n" << Z_k.tail(3) << std::endl;  

	//std::cout << "Z_k " << Z_k << std::endl; 


	Y_k = Z_k - H_k*X_priori; 
	S_k = H_k*P_priori*H_k.transpose() + R_k; 
	K_k = P_priori*H_k.transpose()*S_k.inverse();
	X_posteriori = X_priori + K_k*Y_k; 
	P_posteriori = (I - K_k*H_k)*P_priori; 


	Z_last = Z_k;
	last_measurement_time = input.header.stamp; 
}

void Kalman::publish(){
	estimate.pose.position.x = X_posteriori(0);
	estimate.pose.position.y = X_posteriori(3); 
	estimate.header.stamp=ros::Time::now(); 
	pub.publish(estimate); 
}


/*
	//State estimation 
	Vector3d X_priori, B_k; //Predicated state estimate || Control input model
	Matrix<float,6,6> F_k, Q_k, P_priori;  // State transition model || Covariance process noise 
	// Estimate of accuracy of state estimate before correction

	//Correction
	Vector3d X_posteriori, Y_k; // Corrected state estimate || difference between measurement and prediction
	Matrix<float,6,6>  H_k, R_k, S_k, P_posteriori; //Observation model || Covariance of observation noise || Covariance of observation noise
	// Innovation covariance || Estimate of accuracy of state estimate after correction
*/