#include <iostream>
#include "../Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */  
  VectorXd z_out = VectorXd(3);
  MatrixXd S_out = MatrixXd(3, 3);
  ukf.PredictRadarMeasurement(&z_out, &S_out);

  return 0;
}