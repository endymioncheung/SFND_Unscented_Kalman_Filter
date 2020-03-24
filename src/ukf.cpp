#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
// UKF constructor
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;       // [m/s^2]

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5; // [rad/s^2]
  
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

  // Initialize state vector
  x_ = VectorXd(n_x_);
  x_.fill(0);

  // Initalize covariance matrix with Identity() matrix
  // for faster estimation convergence rather than one
  // for all the matrix
  P_ = MatrixXd::Identity(n_x_,n_x_);
  // P_ = MatrixXd(n_x_, n_x_);
  // P_.setIdentity(n_x_,n_x_);
  
  // Number of sigma points
  n_sig_ =  2 * n_x_ + 1;

  // Number of augmented sigma points
  n_aug_sig_ =  2 * n_aug_ + 1;

  // Augmented mean state
  x_aug_ = VectorXd(n_aug_);

  // Augmented covariance matrix
  P_aug_ = MatrixXd(n_aug_,n_aug_);

  // Augmented sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_,n_aug_sig_);

  // Predicted sigma points matrix as columns in state space
  Xsig_pred_ = MatrixXd(n_x_,n_aug_sig_);

  // Weights of sigma points vector for sigma point
  const double lambda_aug = 3 - n_aug_;
  weights_ = VectorXd(n_aug_sig_);
  weights_(0) = lambda_aug / (lambda_aug + n_aug_);
  for (int i = 1; i < n_aug_sig_ ; ++i) {  
    weights_(i) = 0.5 / (lambda_aug + n_aug_);
  }

  // Radar Measurement dimension: radar can measure r, phi, and r_dot
  n_radar_z_ = 3;

  // Predicted radar measurement mean
  z_radar_pred_ = VectorXd(n_radar_z_);

  // Predicted radar measurement covariance matrix
  S_radar_ = MatrixXd(n_radar_z_,n_radar_z_);

  // Radar measurement noise covariance matrix based on sensor manufacturer
  R_radar_ = MatrixXd(n_radar_z_,n_radar_z_);
  R_radar_ <<  std_radr_*std_radr_,                       0,                     0,
                                 0, std_radphi_*std_radphi_,                     0,
                                 0,                       0, std_radrd_*std_radrd_;
  
  // Radar sigma points matrix in measurement space
  Z_radar_sig_ = MatrixXd(n_radar_z_, n_aug_sig_);

  // Radar cross correlation matrix
  Tc_radar_ = MatrixXd(n_x_, n_radar_z_);
}

// UKF destructor
UKF::~UKF() {}

void UKF::AngleNormalization(VectorXd &angle_vector, int angle_col) {
  // Normalize angle in [rad] 
  while (angle_vector(angle_col) >  M_PI) angle_vector(angle_col) -= 2.*M_PI;
  while (angle_vector(angle_col) < -M_PI) angle_vector(angle_col) += 2.*M_PI;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // 1. Process first measurement from either lidar and radar
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

  // Compute the time elapsed between the current and previous measurements
  // delta time between k and k+1 in seconds
  double delta_t = (meas_package.timestamp_ - time_us_) / 1e6;

  // Update the timestamp from the incoming measurement
  time_us_ = meas_package.timestamp_;

  // 2. Prediction
  Prediction(delta_t);

  // 3. Update lidar or radar measurements
  // // Update lidar measurements if the use_laser_ is enabled
  // if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) UpdateLidar(meas_package);
  // Update radar measurements if the use_radar_ is enabled
  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) UpdateRadar(meas_package);
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // 1. Generate Sigma points
  GenerateSigmaPoints(x_ , P_ , Xsig_);
  AugmentedSigmaPoints(x_aug_, P_aug_, Xsig_aug_);

  // 2. Predict Sigma Points
  SigmaPointPrediction(Xsig_aug_,delta_t,Xsig_pred_);

  // 3. Predict Mean and Convariance
  PredictMeanAndCovariance(Xsig_pred_, x_, P_);
}

void UKF::GenerateSigmaPoints(VectorXd &x, MatrixXd &P, MatrixXd &Xsig) {

  // Spreading parameter
  const double lambda = 3 - n_x_;

  // Calculate square root of P
  // squre root matrix of the covariance matrix P
  // by Cholesky decomposition and taking lower matrix
  // https://en.wikipedia.org/wiki/Cholesky_decomposition
  MatrixXd A = P.llt().matrixL();

  // Sigma point matrix
  Xsig = MatrixXd(n_x_,n_sig_);

  // First column of sigma point matrix
  Xsig.col(0) = x;
  // Set sigma points for the remaining columns of matrix Xsig
  for (int i = 0; i < n_x_ ; ++i) {
    Xsig.col(i+1)      = x + sqrt(lambda + n_x_) * A.col(i);
    Xsig.col(i+1+n_x_) = x - sqrt(lambda + n_x_) * A.col(i);
  }

  // Print result
  // std::cout << "Xsig = " << std::endl << Xsig << std::endl;
}

void UKF::AugmentedSigmaPoints(VectorXd &x_aug, MatrixXd &P_aug, MatrixXd &Xsig_aug) {

  // Spreading parameter
  const double lambda = 3 - n_aug_;

  // Augmented mean state
  x_aug.head(5) = x_;  // first 5 rows of augmented state as original state vector
  x_aug(5)      = 0;   // longitudinal acceleration noise , v_a
  x_aug(6)      = 0;   // yaw acceleration noise, v_psi_ddot

  // Augmented covariance matrix
  // P_aug(a,k|k) = [P_k|k 0]
  //                   [  0   Q]
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;

  // Process noise covariance matrix, Q
  // Q = [std_a_*std_a_           0          ]
  //     [     0      std_yawdd_ * std_yawdd_]
  P_aug(5,5) = std_a_ * std_a_;         // longitudinal acceleration noise
  P_aug(6,6) = std_yawdd_ * std_yawdd_; // yaw acceleration noise

  // Calculate square root of P_aug
  // squre root matrix of the covariance matrix P_aug
  // by Cholesky decomposition and taking lower matrix
  // https://en.wikipedia.org/wiki/Cholesky_decomposition
  MatrixXd A_aug = P_aug.llt().matrixL();

  // Augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda + n_aug_) * A_aug.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda + n_aug_) * A_aug.col(i);
  }
}

void UKF::SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t, MatrixXd &Xsig_pred) {

  const float CLOSE_TO_ZERO = 0.001; // tolerance close to zero

  // Predict sigma points
  for (int i = 0; i < n_aug_sig_; ++i) {
    
    // For better readability - extract values from augmented sigma points matrix 
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
    double yaw_pred = yaw + yaw_dot * delta_t;
    double yaw_dot_pred = yaw_dot;

    // Add process noise to the predicted sigma point
    p_x_pred = p_x_pred + 0.5 * delta_t*delta_t * nu_a * cos(yaw);
    p_y_pred = p_y_pred + 0.5 * delta_t*delta_t * nu_a * sin(yaw);
    v_pred   = v_pred + delta_t * nu_a;
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
}

void UKF::PredictMeanAndCovariance(MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P) {
  
  // Predicted mean state vector
  x.fill(0.0);
  for (int i = 0; i < n_aug_sig_ ; ++i) {  // iterate over sigma points
    x += weights_(i) * Xsig_pred.col(i);
  }

  // Predicted covariance matrix
  P.fill(0.0);
  const int x_idx_yaw = 3;                  // yaw angle index in state vector
  for (int i = 0; i < n_aug_sig_ ; ++i) {  // iterate over sigma points

    // State difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    // Normalize yaw angle (psi) in the state difference within +/- pi
    AngleNormalization(x_diff,x_idx_yaw);

    P +=  weights_(i) * x_diff * x_diff.transpose() ;
  }

  // Print result
  // std::cout << "Predicted state" << std::endl;
  // std::cout << x << std::endl;
  // std::cout << "Predicted covariance matrix" << std::endl;
  // std::cout << P << std::endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // 1. Predict lidar measurement and covariance matrix
  // 2. Update state
  // 3. Calculate NIS (Normalized innovation squared)
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // 1. Predict radar measurement and covariance matrix
  PredictRadarMeasurement(Xsig_pred_, Z_radar_sig_, z_radar_pred_, S_radar_);

  // 2. Update state and radar NIS (Normalized innovation squared)
  UpdateRadarState(meas_package.raw_measurements_, x_, P_, NIS_radar_);
}

void UKF::PredictRadarMeasurement(MatrixXd &Xsig_pred, MatrixXd &Z_radar_sig, VectorXd &z_pred, MatrixXd &S) {

  // Transform radar sigma points into measurement space
  for (int i = 0; i < n_aug_sig_ ; ++i) // 2n+1 simga points
  { 
    // Extract predicted sigma values for better readability
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v   = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);
    // double yaw_dot = Xsig_pred_(4,i);
    double v1 = v * cos(yaw);
    double v2 = v * sin(yaw);

    // Radar measurement model
    Z_radar_sig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Z_radar_sig(1,i) = atan2(p_y,p_x);                                // phi
    Z_radar_sig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // Mean predicted radar measurement
  z_pred.fill(0.0);
  for (int i = 0; i < n_aug_sig_; ++i)
  {
    z_pred += weights_(i) * Z_radar_sig.col(i);
  }

  // Predict radar measurement covariance matrix S
  S.fill(0.0);
  const int z_idx_bearing = 1;  // bearing angle index in radar measurement
  for (int i = 0; i < n_aug_sig_; ++i)
  {
    // Residual
    VectorXd z_diff = Z_radar_sig.col(i) - z_pred;
    // Normalize bearing angle (phi) of residual difference within +/- pi
    AngleNormalization(z_diff,z_idx_bearing);

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // Add radar measurement noise covariance matrix
  S += R_radar_;

  // Print result
  // std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  // std::cout << "S: " << std::endl << S << std::endl;
}

void UKF::UpdateRadarState(VectorXd &z_radar, VectorXd &x, MatrixXd &P, MatrixXd &NIS_radar) {

  // State difference matrix
  VectorXd x_diff;

  // Radar residual difference matrix for intermediate calculation
  VectorXd z_radar_diff;

  // Calculate radar cross correlation matrix 
  // between sigma points in state space and measurement space
  Tc_radar_.fill(0.0);
  const int x_idx_yaw = 3;      // yaw angle index in state vector
  const int z_idx_bearing = 1;  // bearing angle index in radar measurement
  for (int i = 0; i < n_aug_sig_ ; ++i) // 2n+1 simga points
  {
    /* State difference of sigma points
       = predicted sigma points in state space        - mean predicted state        */
    x_diff = Xsig_pred_.col(i) - x;
    // Normalize yaw angle (psi) of sigma points state difference
    AngleNormalization(x_diff,x_idx_yaw);

    /* Residual difference of sigma points
       = measurement sigma points in measurement space - mean predicted measurement */
    z_radar_diff = Z_radar_sig_.col(i) - z_radar_pred_;
    // Normalize bearing angle (phi) of sigma points residual difference
    AngleNormalization(z_radar_diff,z_idx_bearing);

    // Update radar cross correlation matrix
    Tc_radar_ += weights_(i) * x_diff * z_radar_diff.transpose();
  }

  // Calculate radar Kalman gain;
  MatrixXd K_radar = Tc_radar_ * S_radar_.inverse();

  // Update state mean and covariance matrix
  
  /* Measurement residual difference
     = incoming radar measurement - mean predicted measurement */
  z_radar_diff = z_radar - z_radar_pred_;
  // Normalize bearing angle (phi) of measurement residual difference
  AngleNormalization(z_radar_diff,z_idx_bearing);

  // Update state mean and covariance matrix
  x = x + K_radar * z_radar_diff;                    // state
  P = P - K_radar * S_radar_ * K_radar.transpose();  // covariance matrix

  NIS_radar = z_radar_diff.transpose() * S_radar_.inverse() * z_radar_diff;

  // Print result
  // std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  // std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
}