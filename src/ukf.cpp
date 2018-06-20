#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

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
  n_x = x_.size();
  //augmented state dimension
  n_aug = n_x + 2;
  // Number of sigma points
  n_sigma = 2 * n_aug +1;

  //predicted sigma point matrix
  Xsig_predicted = MatrixXd(n_x, n_sigma);

  // Sigma point spreading factor
  lambda = 3 - n_aug;
  // weights of sigma points
  weights = VextorXd(n_sigma);

  // Measurement noise covariance

  R_radar = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radphi_, 0, 0,
		  	  0, 0, std_radphi_*std_radphi_,0,
			  0, 0, std_radrd_;
  R_lidar_ =MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_raspx_,0,
		  	  0,std_laspy_*std_laspy_;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
if (!is_initialized){
	P << 1,0,0,0,0,
		 0,1,0,0,0,
		 0,0,1,0,0,
		 0,0,0,1,0,
		 0,0,0,0,1;
	if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {
		float rho = measurement_pack.raw_measurement[0];
		float phi = measurement_pack.raw_measurement[1];
		float rho_dot = measurement_pack.raw_measurement[2];
		// polar to cartesian
		float px = rho * cos(phi);
		float py = rho * sin(phi);
		float vx = rho_dot * cos(phi);
		float vy = rho_dot * sin(phi);
		float v = sqrt(vx*vx + vy*vy);
		x << px, py, v, 0, 0;
	}
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
		x << measurement_pack.raw_measurement[0], measurement_pack.raw_measurement[1], 0, 0, 0;


	}
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
}
