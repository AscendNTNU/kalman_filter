#include "kalman.h"
#include <iostream>

Kalman::Kalman(){
	//Init vectors and matrices
	x_priori(0) = 0.0; 
	x_priori(1) = 0.0; 
	x_priori(2) = 0.0; 

	x_posteriori(0) = 0.0;
	x_posteriori(1) = 0.0; 
	x_posteriori(2) = 0.0; 
}

void Kalman::printVector(){
	std::cout << x_priori << std::endl; 
}