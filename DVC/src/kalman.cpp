/**
 * Implementation of KalmanFilter class.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <stdexcept>
#include "config.h"
#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    double t0,
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &C,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &P)
    : A(A), C(C), Q(Q), R(R), P0(P),
      m(C.rows()), n(A.rows()), t0(t0), initialized(false),
      I(n, n), x_hat(n), x_hat_new(n), F(n, n)
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

void KalmanFilter::predict(double dt)
{

  // no need to calculate new matrix F as this is linear solution
  if (!initialized)
  {
  }
  else
  {
// Use the state using the state transition matrix
#ifdef DEBUG

    std::cout << "BEFORE PREDICTION" << std::endl
              << "P" << P << std::endl
              << "X" << x_hat << std::endl;

#endif // DEBUG

    this->t += dt;                                   // Adding time so we can track it easier
    this->F = this->I.Identity(n, n) + this->A * dt; // Update F matrix
    x_hat = F * x_hat;
    limit();
    //  Update the covariance matrix using the process noise and state transition matrix
    P = F * P * F.transpose() + Q;

#ifdef DEBUG
    std::cout << "After PREDICTION" << std::endl
              << "P" << P << std::endl
              << "X" << x_hat << std::endl;
#endif // DEBUG
  }
}
void KalmanFilter::update(const Eigen::VectorXd &y_real, const Eigen::VectorXd &y_guess, const Eigen::MatrixXd &C_new)
{

  if (!initialized)
  {
  }
  else
  {
#ifdef DEBUG

    std::cout << "BEFORE UPDATE" << std::endl
              << "C" << C << std::endl
              << "P" << P << std::endl
              << "X" << x_hat << std::endl
              << "YR" << y_real << std::endl
              << "YG" << y_guess << std::endl
              << "K" << K << std::endl;

#endif                                                             // DEBUG
    C << C_new;                                                    // Replace old matrix with updated one
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse(); // Kalmans Matrix
    x_hat_new += K * (y_real - y_guess);                           // Innovation
    x_hat = x_hat_new;                                             // Applying new value to x vector
    P = (I - K * C) * P;                                           // Calculating new P matrix

#ifdef DEBUG

    std::cout << "After UPDATE" << std::endl
              << "F" << F << std::endl
              << "C" << C << std::endl
              << "Q" << Q << std::endl
              << "R" << R << std::endl
              << "P" << P << std::endl
              << "X" << x_hat << std::endl
              << "K" << K << std::endl;

#endif // DEBUG
  }
}


bool KalmanFilter::isInit(void)
{
  return initialized;
}

void KalmanFilter::limit()
{
  if (x_hat(3 - 1) < 0.1)
    x_hat(3 - 1) = 0.1; // limit for Z

  if (x_hat(7 - 1) < 0.1)
    x_hat(7 - 1) = 0.1; // limit for r

  if (x_hat(7 - 1) > 1)
    x_hat(7 - 1) = 1; // limit for r

  if (x_hat(4 - 1) < -30)
    x_hat(4 - 1) = -30; // limit for Vx

  if (x_hat(4 - 1) > 30)
    x_hat(4 - 1) = 30; // limit for Vx

  if (x_hat(5 - 1) < -30)
    x_hat(5 - 1) = -30; // limit for Vy

  if (x_hat(5 - 1) > 30)
    x_hat(5 - 1) = 30; // limit for Vy

  if (x_hat(6 - 1) < -30)
    x_hat(6 - 1) = -30; // limit for Vz

  if (x_hat(6 - 1) > 30)
    x_hat(6 - 1) = 30; // limit for Vz
}