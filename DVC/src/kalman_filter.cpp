#include <iostream>
#include "kalman_filter.h"

#define PI 3.14159265

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;



KalmanFilter::KalmanFilter() {
  init = false;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &C_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  C_ = C_in;
  R_ = R_in;
  Q_ = Q_in;
  init = true;
}

void KalmanFilter::Predict()
{
  // Use the state using the state transition matrix
  x_ = F_ * x_;
  // Update the covariance matrix using the process noise and state transition matrix
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{

  MatrixXd Ct = C_.transpose();
  MatrixXd PCt = P_ * Ct;

  VectorXd y = z - C_ * x_;
  MatrixXd S = C_ * PCt + R_;
  MatrixXd K = PCt * S.inverse();

  // Update State
  x_ = x_ + (K * y);
  // Update covariance matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * C_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &C)
{
  MatrixXd Ct = C.transpose();
  MatrixXd PCt = P_ * Ct;

  VectorXd i = z - C * x_;  //innovation
  MatrixXd S = C * PCt + R_;
  MatrixXd K = PCt * S.inverse();

  // Update State
  x_ = x_ + (K * i);

  // Update covariance matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * C) * P_;
}