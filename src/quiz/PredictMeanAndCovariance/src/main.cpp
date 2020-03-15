#include <iostream>
#include "../../Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */
  
  // sigma point generation
  VectorXd x_pred = VectorXd(5);
  // predicted covariance
  MatrixXd P_pred = MatrixXd(5, 5);

  ukf.PredictMeanAndCovariance(&x_pred, &P_pred);

  return 0;
}