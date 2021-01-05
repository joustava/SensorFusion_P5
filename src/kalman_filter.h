#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "tools.h"
class KalmanFilter {
  Tools tools;
  
  /**
   * @brief Private: Calculate Optimal Kalman gain 
   * 
   * from: https://en.wikipedia.org/wiki/Kalman_filter
   * "The Kalman gain is the relative weight given to the measurements and current state estimate,
   * and can be "tuned" to achieve a particular performance. With a high gain, the filter places more 
   * weight on the most recent measurements, and thus follows them more responsively. 
   * With a low gain, the filter follows the model predictions more closely. At the extremes, 
   * a high gain close to one will result in a more jumpy estimated trajectory, while a low gain close 
   * to zero will smooth out noise but decrease the responsiveness."
   * 
   * @return MatrixXd 
   */
  Eigen::MatrixXd Gain();
  
  /**
   * @brief Private: Calculate new state estimates and updates the filter
   * 
   * @param y VectorXd
   * @return MatrixXd 
   */
  void Estimate(const Eigen::VectorXd &y);

  public:
  /**
   * @brief Construct a new KalmanFilter object
   */
  KalmanFilter();

  /**
   * @brief Destroy the KalmanFilter object
   */
  virtual ~KalmanFilter();

  /**
   * @brief Public: Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * @brief Public: Prediction Predicts the state and the state covariance
   *                using the process model
   * 
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const float delta_T);

  /**
   * @brief Public: Updates the state by using standard Kalman Filter equations
   * 
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * @brief Public: Updates the state by using Extended Kalman Filter equations
   * 
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
};

#endif // _KALMAN_FILTER_H_
