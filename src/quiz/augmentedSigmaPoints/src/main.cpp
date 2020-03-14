#include <iostream>
#include "../../Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */
  MatrixXd Xsig_aug = MatrixXd(7, 15);
  ukf.AugmentedSigmaPoints(&Xsig_aug);

  return 0;
}