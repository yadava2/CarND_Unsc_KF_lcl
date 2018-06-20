#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rsme(4);
	rmse << 0,0,0,0;
	// checking validity of inputs
	if(estimation.size() == 0){
		cout << "Input is empty"<<endl;
		return rmse;
	}
	// The estimation vector size should equal ground truth vector size
	if(estimation.size() != ground_truth.size()){
		cout<<"Invalid estimation or ground truth";
		return rmse;
	}

	// Accumulate squared residuals
	for(unsigned int i=0; i < estimations.size();++i){
		VectorXd residual = estimations[i] - ground_truth[i];

		residual = residual.array()*residual.array();
		rmse += residual;
	}
	// Calculate the mean
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;

}

