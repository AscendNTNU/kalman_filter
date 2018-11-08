#ifndef _KALMAN_H_
#define _KALMAN_H_


#include <Eigen/Dense>

using namespace Eigen; 

class Kalman{

public:
	Kalman();
	Kalman(int var); 
	void printVector();
	void prediction(); //Callback? 
	void correction(); //Callback!
	
private:
	//State estimation 
	Matrix<float, 6, 1> X_priori, B_k; //Predicated state estimate || Control input model
	Matrix<float,6,6> F_k, Q_k, P_priori;  // State transition model || Covariance process noise 
	// Estimate of accuracy of state estimate before correction

	//Correction
	Matrix<float, 6, 1> X_posteriori, Y_k; // Corrected state estimate || difference between measurement and prediction
	Matrix<float,6,6>  H_k, R_k, S_k, P_posteriori; //Observation model || Covariance of observation noise || Covariance of observation noise
	// Innovation covariance || Estimate of accuracy of state estimate after correction

};

#endif