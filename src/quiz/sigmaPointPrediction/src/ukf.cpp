#include "ukf.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}


/**
 * Programming assignment functions: 
 */

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  // set state dimension
  const int n_x = 5;

  // set augmented dimension
  const int n_aug = 7;

  // create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  const double delta_t = 0.1; // time diff in sec

  /**
   * Student part begin
   */

  const float CLOSE_TO_ZERO = 0.001;
  // predict sigma points
  for (int i = 0; i< 2*n_aug + 1; ++i) {
    
    // extract values for better readability
    double p_x         = Xsig_aug(0,i); // x-position
    double p_y         = Xsig_aug(1,i); // y-positiom
    double v           = Xsig_aug(2,i); // velocity
    double yaw         = Xsig_aug(3,i); // yaw angle
    double yaw_dot     = Xsig_aug(4,i); // yaw rate
    double nu_a        = Xsig_aug(5,i); // acceleration noise
    double nu_yaw_ddot = Xsig_aug(6,i); // yaw rate noise

    // predict x and y position of the state values
    double p_x_pred, p_y_pred;
    // avoid division by zero when zero yaw rate (`yaw_dot`) is used
    // for predicting the x and y position of sigma point
    if (fabs(yaw_dot) > CLOSE_TO_ZERO) {
        // predict x and y when yaw rate is not zero
        p_x_pred = p_x + v/yaw_dot * ( sin(yaw + yaw_dot*delta_t) - sin(yaw) );
        p_y_pred = p_y + v/yaw_dot * ( cos(yaw) - cos(yaw+yaw_dot*delta_t) );
    } else {
        // special case when yaw rate is zero
        p_x_pred = p_x + v * delta_t * cos(yaw);
        p_y_pred = p_y + v * delta_t * sin(yaw);
    }

    // predict the velocity, yaw amgle and yaw rate of sigma point
    double v_pred = v;
    double yaw_pred = yaw + yaw_dot*delta_t;
    double yaw_dot_pred = yaw_dot;

    // add process noise to the predicted sigma point
    p_x_pred = p_x_pred + 0.5 * delta_t*delta_t * nu_a * cos(yaw);
    p_y_pred = p_y_pred + 0.5 * delta_t*delta_t * nu_a * sin(yaw);
    v_pred = v_pred + delta_t * nu_a;
    yaw_pred = yaw_pred + 0.5 * delta_t*delta_t * nu_yaw_ddot;
    yaw_dot_pred = yaw_dot_pred + delta_t * nu_yaw_ddot;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = p_x_pred;
    Xsig_pred(1,i) = p_y_pred;
    Xsig_pred(2,i) = v_pred;
    Xsig_pred(3,i) = yaw_pred;
    Xsig_pred(4,i) = yaw_dot_pred;
  }
  
  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;
}

/** 
 * expected result:
 *  Xsig_aug =
 * 5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
 *   1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
 * 2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
 * 0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
 * 0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
 *      0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
 *      0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
 */