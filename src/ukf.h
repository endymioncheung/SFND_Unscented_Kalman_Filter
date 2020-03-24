#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  void AngleNormalization(Eigen::VectorXd &angle_vector, int angle_col);
  void GenerateSigmaPoints(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &Xsig);
  void AugmentedSigmaPoints(Eigen::VectorXd &x_aug, Eigen::MatrixXd &P_aug, Eigen::MatrixXd &Xsig_aug);
  void SigmaPointPrediction(Eigen::MatrixXd &Xsig_aug, double delta_t, Eigen::MatrixXd &Xsig_pred);
  void PredictMeanAndCovariance(Eigen::MatrixXd &Xsig_pred, Eigen::VectorXd &x, Eigen::MatrixXd &P);
  
  void PredictRadarMeasurement(Eigen::MatrixXd &Xsig_pred, Eigen::MatrixXd &Z_radar_sig, Eigen::VectorXd &z_pred, Eigen::MatrixXd &S);
  void UpdateRadarState(Eigen::VectorXd &z_radar, Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &NIS_radar);

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // Time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Number of sigma points
  int n_sig_;

  // Number of augmented points
  int n_aug_sig_;

  // Sigma points matrix
  Eigen::MatrixXd Xsig_;

  // Augmented mean state
  Eigen::VectorXd x_aug_;

  // Augmented covariance matrix
  Eigen::MatrixXd P_aug_ ;

  // Augmented sigma point matrix
  Eigen::MatrixXd Xsig_aug_;

  // Predicted sigma points matrix as columns in state space
  Eigen::MatrixXd Xsig_pred_;

  // Weights of sigma points vector for sigma point
  Eigen::VectorXd weights_;

  // Measurement dimension, radar can measure r, phi, and r_dot
  int n_radar_z_;
  
  // Predicted radar measurement mean
  Eigen::VectorXd z_radar_pred_;
  
  // Predicted radar measurement covariance matrix
  Eigen::MatrixXd S_radar_;

  // Radar measurement noise covariance matrix
  Eigen::MatrixXd R_radar_;

  // Radar sigma points matrix in measurement space
  Eigen::MatrixXd Z_radar_sig_;

  // Radar cross correlation matrix
  Eigen::MatrixXd Tc_radar_;

  // Radar NIS (Normalized innovation squared)
  Eigen::MatrixXd NIS_radar_;
};

#endif  // UKF_H