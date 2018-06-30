#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include "tools.h"

#define EPS 0.001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  // State dimension
  n_x_ = x_.size();

  //augmented state dimension
  n_aug_ = n_x_ + 2;

  // Number of sigma points
  n_sig_ = 2 * n_aug_ +1;

  //predicted sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Sigma point spreading factor
  lambda_ = 3 - n_aug_;

  // weights of sigma points
  weights_ = VectorXd(n_sig_);

  // Measurement noise covariance

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
		  	  0, std_radphi_*std_radphi_, 0,
			  0, 0, std_radrd_;
  R_lidar_ =MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_,0,
		  	  0,std_laspy_*std_laspy_;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */

//Angle Normalization

void UKF::NormAng(double *ang){
	while(*ang > M_PI) *ang -= 2. * M_PI;
	while(*ang < M_PI) *ang += 2. * M_PI;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	if (!is_initialized_){
		P_ << 1,0,0,0,0,
			  0,1,0,0,0,
			  0,0,1,0,0,
			  0,0,0,1,0,
			  0,0,0,0,1;

	// Intialisation for RADAR, for CTRV model x=[px, py, vel, ang, ang_rate]
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		float rho = meas_package.raw_measurements_[0];
		float phi = meas_package.raw_measurements_[1];
		float rho_dot = meas_package.raw_measurements_[2];
		// polar to cartesian
		float px = rho * cos(phi);
		float py = rho * sin(phi);
		float vx = rho_dot * cos(phi);
		float vy = rho_dot * sin(phi);
		float v = sqrt(vx*vx + vy*vy);
		x_ << px, py, v, 0, 0;
	  }
	// Initialisation for LIDAR
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
		    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
	       }



// weights initialisation

	weights_(0) = lambda_ / (lambda_ + n_aug_);
	for(int i=1; i<weights_.size(); i++){
		weights_(i)=0.5/(lambda_ + n_aug_);
	}

	time_us_ = meas_package.timestamp_;

	is_initialized_ = true;

	return;
}

// calculation of timestamp
	double dt = (meas_package.timestamp_ - time_us_);
	dt /= 1000000.0;    //converting into seconds
	time_us_ = meas_package.timestamp_;
	Prediction(dt);

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
		UpdateRadar(meas_package);
	}

	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
			UpdateLidar(meas_package);
		}

}



/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	double delta_t2 =delta_t * delta_t;

	//1. Generating Sigma Points

	//Augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);

	//Augmented state covarience matrix
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	//sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

	//Initializing the matrices
	x_aug.fill(0.0);
	x_aug.head(n_x_) = x_;
	P_aug.fill(0);
	P_aug.topLeftCorner(n_x_,n_x_) = P_;
	P_aug(5,5) = std_a_ *std_a_;
	P_aug(6,6) = std_yawdd_ * std_yawdd_;

	// square root of P matrix

	MatrixXd L = P_aug.llt().matrixL();

	//Create sigma points

	Xsig_aug.col(0) = x_aug;
	double sqrt_lambda_n_aug = sqrt(lambda_ + n_aug_);

	VectorXd sqrt_lambda_n_aug_L;
	for(int i=0; i<n_aug_; i++){
		// multiplication of the A and sqrt_lamda_n_aug
		sqrt_lambda_n_aug_L = sqrt_lambda_n_aug * L.col(i);
		Xsig_aug.col(i+1) = x_aug + sqrt_lambda_n_aug_L;
		Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_lambda_n_aug_L;
	}

	// Prediction of Sigma Points
	for (int i=0; i< n_sig_ ; i++)
	{
		// Extracting the values for better readability
		double p_x = Xsig_aug(0,i);
		double p_y = Xsig_aug(1,i);
		double v = Xsig_aug(2,i);
		double yaw = Xsig_aug(3,i);
		double yawd = Xsig_aug(4,i);
		double nu_a = Xsig_aug(5,i);   //augmented variables
		double nu_yawdd = Xsig_aug(6,i);

		// Calculating sin and cos optimizations
		double sin_yaw = sin(yaw);
		double cos_yaw = cos(yaw);
		double arg = yaw + yawd*delta_t;

		// state Values

		double px_p, py_p;
		// Avoid division by zero

		if (fabs(yawd)> EPS){
			double v_yawd = v/yawd;
			px_p = p_x + v_yawd * (sin(arg) - sin_yaw);
			px_p = p_y + v_yawd * (cos_yaw - cos(arg));
		}
		else {
			double v_delta_t = v*delta_t;
			px_p = p_x + v_delta_t*cos_yaw;
			px_p = p_x + v_delta_t*sin_yaw;

		}

		double v_p = v;
		double yaw_p = arg;
		double yawd_p= yawd;

		// adding noise

		px_p += 0.5*nu_a*delta_t2*cos_yaw;
		py_p += 0.5*nu_a*delta_t2*sin_yaw;
		v_p += nu_a*delta_t;
		yaw_p += 0.5*nu_yawdd*delta_t2;
		yawd_p += nu_yawdd*delta_t;

		// Writing the sigma points

		Xsig_pred_(0,i) = px_p;
		Xsig_pred_(1,i) = py_p;
		Xsig_pred_(2,i) = v_p;
		Xsig_pred_(3,i) = yaw_p;
		Xsig_pred_(4,i) = yawd_p;

	}

	//Prediction of Mean and covariance

	//Mean

	x_ = Xsig_pred_ * weights_;

	// Covariance

	P_.fill(0.0);
	for(int i=0; i< n_sig_; i++){
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		NormAng(&(x_diff(3)));
		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}

}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	int n_z = 2;
		MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);
		UpdateUKF(meas_package, Zsig, n_z);



}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
		// measurement dimension
		int n_z=3;

		//sigma point matrix
		MatrixXd Zsig = MatrixXd(n_z, n_sig_);

		//transforming sigma points into measurement space
		for(int i=0; i<n_sig_; i++){
			double p_x = Xsig_pred_(0,i);
			double p_y = Xsig_pred_(1,i);
			double v = Xsig_pred_(2,i);
			double yaw = Xsig_pred_(3,i);

			double v1 = cos(yaw)*v;
			double v2 = sin(yaw)*v;
			// Measurement model
			Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
			Zsig(1,i) = atan2(p_y,p_x);
			Zsig(2,i) = (p_x*v1 + p_y*v2)/ Zsig(0,i);
		}
		UpdateUKF(meas_package, Zsig, n_z);
}

void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z){

	//1. Predict Measurement
	// Mean prediction measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred = Zsig * weights_;

	// S : Measurement Covariance Matrix
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for(int i=0; i < n_sig_; i++){
		// residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		// angle normalization
		NormAng(&(z_diff(1)));
		S = S + weights_(i) * z_diff *z_diff.transpose();
	}

	//Adding Measurement Noise Covariance Matrix
	MatrixXd R =  MatrixXd(n_z, n_z);
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
		R=R_radar_;
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
		R=R_lidar_;
}
	// add measurement noise covariance matrix
	S=S+R;

	//2. Update State

	//Incoming Measurement
		VectorXd z = meas_package.raw_measurements_;

	//Cross Corelation Tc

	MatrixXd Tc = MatrixXd(n_x_,n_z);
	Tc.fill(0.0);

	for(int i=0; i < n_sig_; i++){
		VectorXd z_diff = Zsig.col(i) - z_pred;
		if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
			// Angle normalization
			NormAng(&(z_diff(1)));
		}
		// State Difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// Angle normalization
		NormAng(&(x_diff(3)));
		Tc = Tc + weights_(i) * x_diff *z_diff.transpose();
		}

	//Kalman Gain
	MatrixXd K = Tc * S.inverse();

	//Residual
	VectorXd z_diff = z - z_pred;
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
		// Angle Normalization
		NormAng(&(z_diff(1)));
	}

	//Updating state mean and covariance matrix
	x_ = x_ + K*z_diff;
	P_ = P_ - K * S * K.transpose();
	// Calculating Normalized Innovation Squared
	if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
		NIS_radar_ = z.transpose() * S.inverse() * z;
	}
	else if(meas_package.sensor_type_ == MeasurementPackage::LASER){
			NIS_laser_ = z.transpose() * S.inverse() * z;
		}


 }



