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

  // predicted measurement mean, z_out
  VectorXd z_out = VectorXd(3);

  // predicted measurement covariance matrix, S_out
  MatrixXd S_out = MatrixXd(3, 3);
  ukf.PredictRadarMeasurement(&z_out, &S_out);

  return 0;
}