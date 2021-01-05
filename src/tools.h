#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
  public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * @brief Calculate Root Mean Square Error
   * 
   * @param estimations std::vector<Eigen::VectorXd>
   * @param ground_truth std::vector<Eigen::VectorXd>
   * @return Eigen::VectorXd 
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * @brief Calculate Jacobian matrix to handle non-linearity in Radar measurement
   * 
   * @param x_state Eigen::VectorXd
   * @return Eigen::MatrixXd a Jacobian matrix
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

  /**
   * @brief Public: Convert cartesian measurment vector to polar vector.
   * 
   * @param x_state Eigen::VectorXd (px​, py​, vx​, vy)T
   * @return Eigen::VectorXd (ρ,φ,ρ_dot)T
   */
  Eigen::VectorXd ConvertCartesianToPolar(const Eigen::VectorXd &x_state);
};

#endif // _TOOLS_H_
