#ifndef _FusionEKF_H_
#define _FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
  public:
  /**
   * @brief Construct a new Fusion E K F object
   * 
   */
  FusionEKF();

  /**
   * @brief Destroy the Fusion E K F object
   * 
   */
  virtual ~FusionEKF();

  /**
   * @brief Public: Handle a complete filter cycle, initializes for you.
   * 
   * @param measurement_pack 
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * @brief Kalman Filter update and prediction math lives in here.
   * 
   */
  KalmanFilter ekf_;

  private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  float noise_ax;
  float noise_ay;
  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  
  /**
   * @brief Private: Initialise EKF when needed.
   * 
   * @param measurement_pack 
   */
  void ProcessInit(const MeasurementPackage &measurement_pack);
  
  /**
   * @brief Private: Handle EKF prediction step.
   * 
   * @param measurement_pack 
   */
  void ProcessPrediction(const MeasurementPackage &measurement_pack);
  
  /**
   * @brief Private: Handle EKF update step.
   * 
   * @param measurement_pack 
   */
  void ProcessUpdate(const MeasurementPackage &measurement_pack);
};

#endif // FusionEKF_H_
