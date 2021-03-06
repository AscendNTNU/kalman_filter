#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>

using namespace Eigen; 

class Kalman{

public:
	Kalman();
	void printSystem();
	void prediction();
	void correction(const geometry_msgs::PoseStamped& input); //Callback
	void publish(); 
	void updateOrientation(const geometry_msgs::PoseStamped& input); //Callback roll, pitch, yaw
	
private:
	//State estimation 
	Matrix<float,3,3> rotation_matrix; 
	Matrix<float,3,1> orientation_rpy; //Orientation given in roll pitch yaw! 
	Matrix<float,6,1> X_priori, B_k; //Predicated state estimate || Control input model
	Matrix<float,6,6> F_k, F_k_inv, Q_k, P_priori, I;  // State transition model || Covariance process noise 
	// Pre covariance of state variables || Idenity matrix

	//Correction
	Matrix<float,6,1> X_posteriori, Z_last; // Corrected state estimate || difference between measurement and prediction || measurement
	Matrix<float,6,6>  P_posteriori; //Kalman gian || Observation model || Covariance of observation noise || Rest-covariance || Post covariance of state variables

	Matrix<float,6,3> K_k; 

	Matrix<float,3,3> R_k, S_k; 
	Matrix<float,3,6> H_k; 
	Matrix<float,3,1> Y_k, Z_k;
	//First iteration 
	Matrix<float,4,1> q; // quaternion
	bool first_iteration; 
	ros::Time last_measurement_time; 

	//Publisher! 
	ros::Publisher pub_position;
	ros::Publisher pub_velocity;
	//Estimates 
	geometry_msgs::PoseStamped pose_estimate;
	geometry_msgs::TwistStamped twist_estimate; 
};

#endif