#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
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
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

   // Default initialization to false and set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;
  
  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Initalize 1x5 state vector
  x_.fill(0);

  // Set 5x5 covariance matrix to identity matrix
  P_.setIdentity(n_x_,n_x_);

  // set vector for weights of sigma points
  double weight = 0.5 / (lambda_+n_aug_);
  weights_(0)   = lambda_/(lambda_+n_aug_);;

  for (int i=1; i < 2*n_aug_ + 1; ++i) {  
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // state vector, x_ expects [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad

  // Set measurement timestamp
  time_us_ = meas_package.timestamp_ ;

  // Process first measurement from either lidar and radar
  if (!is_initialized_) {

    // Process measurement from lidar sensor
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      double px = meas_package.raw_measurements_(0);       // x-position
      double py = meas_package.raw_measurements_(1);       // y-position
      x_ << px, py, 0, 0, 0;
    }
    // Processs measurement from radar sensor
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_(0);      // range
      double phi = meas_package.raw_measurements_(1);      // bearing
      double rho_dot = meas_package.raw_measurements_(2);  // change of range

      // Transform radar's polar coordinate to Cartesian coordinate
      double px = rho * cos(phi);      // x-position
      double py = rho * sin(phi);      // y-position
      double vx = rho_dot * cos(phi);  // x-component of velocity
      double vy = rho_dot * sin(phi);  // y-component of velocity
      double v = sqrt(vx*vx + vy*vy);  // absolute velocity
      x_ << px, py, v, 0, 0;
    } else {
      std::cerr << "Invalid sensor type!" << std::endl;
      return;
    }

    // First call measurement initalization completed
    is_initialized_ = true;
    return;
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Estimate the object's location
  // Modify state vector, x_
  // Predict sigma points
  // Predict state
  // Predict state covariance matrix
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}