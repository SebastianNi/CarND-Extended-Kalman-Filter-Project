#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

  // Just a draft of the implementation
  /*x = F * x + u;
  MatrixXd Ft = F.transpose();
  P = F * P * Ft + Q;*/
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // Just a draft of the implementation
  /*VectorXd y = z - H * x;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P * Ht * Si;

  //new state
  x = x + (K * y);
  P = (I - K * H) * P;*/
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
