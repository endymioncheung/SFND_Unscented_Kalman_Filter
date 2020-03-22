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

  // Number of sigma points
  n_sig_ =  2 * n_x_ + 1;

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

    // Set timestamp for first measurement
    time_us_ = meas_package.timestamp_;
    // First call measurement initalization completed
    is_initialized_ = true;    
    return;
  }

  // Prediction
  // Compute the time elapsed between the current and previous measurements
  // delta_t - expressed in seconds
  double delta_t = (meas_package.timestamp_ - time_us_) / 1e6; // delta time between k and k+1 in seconds
  Prediction(delta_t);
  // Update the timestamp from the new measurement
  time_us_ = meas_package.timestamp_;

  // Update lidar measurements if the use_laser_ is enabled
  if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
    std::cout << "Update laser measurements" << std::endl;
    UpdateLidar(meas_package);
  }
  
  // Update radar measurements if the use_radar_ is enabled
  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    std::cout << "Update radar measurements" << std::endl;
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Modify state vector as augmented state vector

  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_; // set first 5 elements of augmented state as original state
  x_aug(5) = 0;       // v_a
  x_aug(6) = 0;       // v_psi_ddot

  // Augmented state covariance matrix
  // P_aug_(a,k|k) = [P_k|k 0]
  //                 [  0   Q]              
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);  
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;

  // Process noise covariance matrix, Q
  // Q = [std_a_*std_a_           0          ]
  //     [     0      std_yawdd_ * std_yawdd_]
  P_aug(5,5) = std_a_ * std_a_;         // longitudinal acceleration noise
  P_aug(6,6) = std_yawdd_ * std_yawdd_; // yaw acceleration noise

  // 1. Generate Sigma points
  MatrixXd Xsig_aug = GenerateSigmaPoints(x_aug,P_aug,lambda_);

  // 2. Predict Sigma Points
  Xsig_pred_ = SigmaPointPrediction(Xsig_aug,delta_t);

  // 3. Predict Mean and Convariance
  PredictMeanAndCovariance(Xsig_pred_,x_,P_);
}

Eigen::MatrixXd UKF::GenerateSigmaPoints(VectorXd &x, MatrixXd &P, double lambda) {

  // Calculate square root of P
  // squre root matrix of the covariance matrix P
  // by Cholesky decomposition and taking lower matrix
  // https://en.wikipedia.org/wiki/Cholesky_decomposition
  MatrixXd A = P.llt().matrixL();

  // Sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_,n_sig_); // 5 rows and 11 columns  
  // Set first column of sigma point matrix
  Xsig.col(0) = x;
  // Set sigma points for the remaining columns of matrix Xsig
  for (int i = 0; i < n_x_; ++i) {
    Xsig.col(i+1)     = x + sqrt(lambda+n_x_) * A.col(i);
    Xsig.col(i+1+n_x_) = x - sqrt(lambda+n_x_) * A.col(i);
  }

  // Print result
  // std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  return Xsig;
}

Eigen::MatrixXd UKF::SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t) {

  const float CLOSE_TO_ZERO = 0.001; // tolerance close to zero

  // Matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Predict sigma points
  for (int i = 0; i< 2*n_aug_ + 1; ++i) {
    
    // Extract values for better readability
    double p_x         = Xsig_aug(0,i); // x-position
    double p_y         = Xsig_aug(1,i); // y-position
    double v           = Xsig_aug(2,i); // velocity
    double yaw         = Xsig_aug(3,i); // yaw angle, psi
    double yaw_dot     = Xsig_aug(4,i); // yaw rate, psi dot
    double nu_a        = Xsig_aug(5,i); // acceleration noise
    double nu_yaw_ddot = Xsig_aug(6,i); // yaw rate noise

    // Predict x and y position of the state values, and
    // avoid division by zero when zero yaw rate (`yaw_dot`)
    double p_x_pred, p_y_pred;
    if (fabs(yaw_dot) > CLOSE_TO_ZERO) {
        // predict x and y when yaw rate is not zero
        p_x_pred = p_x + v/yaw_dot * ( sin(yaw + yaw_dot*delta_t) - sin(yaw) );
        p_y_pred = p_y + v/yaw_dot * ( cos(yaw) - cos(yaw+yaw_dot*delta_t) );
    } else {
        // special case when yaw rate is zero
        p_x_pred = p_x + v * delta_t * cos(yaw);
        p_y_pred = p_y + v * delta_t * sin(yaw);
    }

    // Predict velocity, yaw angle and yaw rate of the sigma point
    double v_pred = v;
    double yaw_pred = yaw + yaw_dot*delta_t;
    double yaw_dot_pred = yaw_dot;

    // Add process noise to the predicted sigma point
    p_x_pred = p_x_pred + 0.5 * delta_t*delta_t * nu_a * cos(yaw);
    p_y_pred = p_y_pred + 0.5 * delta_t*delta_t * nu_a * sin(yaw);
    v_pred = v_pred + delta_t * nu_a;
    yaw_pred = yaw_pred + 0.5 * delta_t*delta_t * nu_yaw_ddot;
    yaw_dot_pred = yaw_dot_pred + delta_t * nu_yaw_ddot;

    // Write predicted sigma point into right column
    Xsig_pred(0,i) = p_x_pred;
    Xsig_pred(1,i) = p_y_pred;
    Xsig_pred(2,i) = v_pred;
    Xsig_pred(3,i) = yaw_pred;
    Xsig_pred(4,i) = yaw_dot_pred;
  }

  // Print result
  // std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  return Xsig_pred;
}

void UKF::PredictMeanAndCovariance(MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P) {

  // Weights for each sigma point
  VectorXd weights = VectorXd(n_sig_);
  const double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i < n_sig_; ++i) {
    double weight = 0.5 / (lambda_+n_aug_);
    weights(i) = weight;
  }

  // Predicted mean
  x.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {  // iterate over sigma points
    x += weights(i) * Xsig_pred.col(i);
  }

  // Predicted covariance
  P.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {  // iterate over sigma points
    // State difference
    VectorXd x_diff = Xsig_pred.col(i) - x;

    // Normalize yaw angle within +/- pi
    while (x_diff(3) >  M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    P +=  weights(i) * x_diff * x_diff.transpose() ;
  }

  // Print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;
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