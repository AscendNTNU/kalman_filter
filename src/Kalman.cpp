#include "kalman.h"
#include <iostream>
#include <Eigen/Dense>
Kalman::Kalman(){
	//Init vectors and matrices
	X_priori.setZero(); 
	X_posteriori.setZero(); 
	F_k.setZero(); 

};

Kalman::Kalman(int var){
	X_priori << var, var, var; 
};

void Kalman::printVector(){	
	std::cout << F_k << std::endl; 
};

void Kalman::prediction(){
	X_priori = F_k*X_posteriori; // + B_k U 
	P_priori = F_k*P_posteriori*F_k.transpose() + Q_k; 
}

void Kalman::correction(){
	
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