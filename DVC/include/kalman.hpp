/**
 * Kalman filter implementation using Eigen. Based on the following
 * introductory paper:
 *
 *     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <Eigen/Dense>
#pragma once

class KalmanFilter
{

public:
  /**
   * Create a Kalman filter with the specified matrices.
   *   A - System dynamics matrix
   *   C - Output matrix
   *   Q - Process noise covariance
   *   R - Measurement noise covariance
   *   P - Estimate error covariance
   */
  KalmanFilter(
      double dt,
      const Eigen::MatrixXd &F,
      const Eigen::MatrixXd &C,
      const Eigen::MatrixXd &Q,
      const Eigen::MatrixXd &R,
      const Eigen::MatrixXd &P);

  /**
   * Create a blank estimator.
   */
  KalmanFilter();

  /**
   * Initialize the filter with initial states as zero.
   */
  void init();

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(double t0, const Eigen::VectorXd &x0);

  /**
   * @brief Is it initialized
   *
   * @return true
   * @return false
   */

  bool isInit();

  /**
   * @brief Full initialization
   *
   * @param t0
   * @param x0
   * @param F_in
   * @param C_in
   * @param Q_in
   * @param R_in
   * @param P_in
   */
  void init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &F_in,
            const Eigen::MatrixXd &C_in,
            const Eigen::MatrixXd &Q_in,
            const Eigen::MatrixXd &R_in,
            const Eigen::MatrixXd &P_in);

  void predict();
  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */
  void update(const Eigen::VectorXd &y_real, const Eigen::VectorXd &y_guess, const Eigen::MatrixXd &C_new);

  /**
   * Update the estimated state based on measured values,
   * using the given time step and dynamics matrix.
   */
  void update(const Eigen::VectorXd &y_real, const Eigen::VectorXd &y_guess, double dt, const Eigen::MatrixXd &C_new);

  /**
   * Return the current state and time.
   */
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };

private:
  // Matrices for computation
  Eigen::MatrixXd F, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};