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

  /**
   * Normalize the angle within +/- pi
   * @param angle_vector Vector containing angle in rad
   * @param angle_col    column index of the angle
   */
  void AngleNormalization(Eigen::VectorXd &angle_vector, int angle_col);

  /**
   * Generate the sigma point matrix for given state vector and covariance matrix
   * @param x State vector
   * @param P Covariance matrix
   * @param x Sigma point matrix
   */
  void GenerateSigmaPoints(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &Xsig);

  /**
   * Augment sigma points
   * @param x_aug     Augmented mean state
   * @param P_aug     Augmented covariance matrix
   * @param X_sig_aug Augmented sigma point matrix
   */
  void AugmentedSigmaPoints(Eigen::VectorXd &x_aug, Eigen::MatrixXd &P_aug, Eigen::MatrixXd &Xsig_aug);

  /**
   * Sigma points prediction
   * @param X_sig_aug Augmented sigma point matrix
   * @param deta_t    Time between k and k+1 in s
   * @param Xsig_pred Predicted sigma points matrix as columns in state space
   */
  void SigmaPointPrediction(Eigen::MatrixXd &Xsig_aug, double delta_t, Eigen::MatrixXd &Xsig_pred);

  /**
   * Predict mean and covariance matrix
   * @param Xsig_pred Predicted sigma points matrix as columns in state space
   * @param x_aug     Augmented mean state
   * @param P_aug     Augmented covariance matrix 
   */
  void PredictMeanAndCovariance(Eigen::MatrixXd &Xsig_pred, Eigen::VectorXd &x, Eigen::MatrixXd &P);
  
  /**
   * Predict radar measurement
   * @param Xsig_pred   Predicted sigma points matrix as columns in state space
   * @param Z_radar_sig Radar sigma points matrix in measurement space
   * @param z_pred      Predicted radar measurement mean
   * @param S           Predicted radar measurement covariance matrix
   */
  void PredictRadarMeasurement(Eigen::MatrixXd &Xsig_pred, Eigen::MatrixXd &Z_radar_sig, Eigen::VectorXd &z_pred, Eigen::MatrixXd &S);

  /**
   * Update radar state
   * @param z_radar   Radar measurement
   * @param x         Radar sigma points matrix in measurement space
   * @param P         Predicted radar measurement mean
   * @param NIS_radar Predicted radar measurement covariance matrix
   */
  void UpdateRadarState(Eigen::VectorXd &z_radar, Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &NIS_radar);

  /**
   * Predict lidar measurement
   * @param Xsig_pred   Predicted sigma points matrix as columns in state space
   * @param Z_lidar_sig Lidar sigma points matrix in measurement space
   * @param z_pred      Predicted lidar measurement mean
   * @param S           Predicted lidar measurement covariance matrix
   */
  void PredictLidarMeasurement(Eigen::MatrixXd &Xsig_pred, Eigen::MatrixXd &Z_lidar_sig, Eigen::VectorXd &z_pred, Eigen::MatrixXd &S);

  /**
   * Update lidar state
   * @param z_liar    Lidar measurement
   * @param x         Lidar sigma points matrix in measurement space
   * @param P         Predicted lidar measurement mean
   * @param NIS_lidar Predicted lidar measurement covariance matrix
   */
  void UpdateLidarState(Eigen::VectorXd &z_lidar, Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &NIS_lidar);

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

  /****************************************
  | Radar measurement update attributes  *|
  ****************************************/

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

  /****************************************
  | Lidar measurement update attributes  *|
  ****************************************/

  // Measurement dimension, lidar can measure x, y
  int n_lidar_z_;

  // Predicted lidar measurement mean
  Eigen::VectorXd z_lidar_pred_;

  // Predicted lidar measurement covariance matrix
  Eigen::MatrixXd S_lidar_;

  // Lidar measurement noise covariance matrix
  Eigen::MatrixXd R_lidar_;

  // Lidar sigma points matrix in measurement space
  Eigen::MatrixXd Z_lidar_sig_;

  // Lidar cross correlation matrix
  Eigen::MatrixXd Tc_lidar_;

  // Lidar NIS (Normalized innovation squared)
  Eigen::MatrixXd NIS_lidar_;
};

#endif  // UKF_H