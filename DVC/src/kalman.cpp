/**
 * Implementation of KalmanFilter class.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &C,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &P)
    : A(A), C(C), Q(Q), R(R), P0(P),
      m(C.rows()), n(A.rows()), dt(dt), initialized(false),
      I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &A_in,
                        const Eigen::MatrixXd &C_in,
                        const Eigen::MatrixXd &Q_in,
                        const Eigen::MatrixXd &R_in,
                        const Eigen::MatrixXd &P_in)
{
  x_hat = x0;
  P = P0;
  A = A_in;
  C = C_in;
  Q = Q_in;
  R = R_in;
  P = P_in;
  initialized = true;
}

void KalmanFilter::init()
{
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::predict()
{
  if (!initialized)
  {
  }
  else
  {
    // Use the state using the state transition matrix
    x_hat = A * x_hat;
    // Update the covariance matrix using the process noise and state transition matrix
    P = A * P * A.transpose() + Q;
  }
}
void KalmanFilter::update(const Eigen::VectorXd &y, const Eigen::MatrixXd &C_new)
{

  if (!initialized)
  {
  }
  else
  {
    /*
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
  */
    // x_hat_new = A * x_hat;
    C = C_new;
    P = A * P * A.transpose() + Q;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat_new += K * (y - C * x_hat_new);
    P = (I - K * C) * P;
    x_hat = x_hat_new;

  }
}

void KalmanFilter::update(const Eigen::VectorXd &y, double dt, const Eigen::MatrixXd A, const Eigen::MatrixXd &C_new)
{

  this->A = A;
  update(y, C_new);
}

bool KalmanFilter::isInit(void)
{
  return initialized;
}