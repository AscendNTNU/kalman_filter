#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>


using namespace Eigen; 

class Kalman{

public:
	Kalman();
	void printVector();
	void prediction(); //Callback? 
	void correction(const geometry_msgs::PoseStamped& input); //Callback!
	
private:
	//State estimation 
	Matrix<float, 6, 1> X_priori, B_k; //Predicated state estimate || Control input model
	Matrix<float,6,6> F_k, Q_k, P_priori, I;  // State transition model || Covariance process noise 
	// Pre covariance of state variables || Idenity matrix

	//Correction
	Matrix<float, 6, 1> X_posteriori, Y_k, Z_k; // Corrected state estimate || difference between measurement and prediction || measurement
	Matrix<float,6,6>  K_k, H_k, R_k, S_k, P_posteriori; //Kalman gian || Observation model || Covariance of observation noise || Rest-covariance || Post covariance of state variables
};

#endif