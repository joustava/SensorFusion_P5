#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd> &estimations,
                              const std::vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.empty()) {
      std::cout << "the estimation vector size should not be zero" << std::endl;
      return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
      std::cout << "the estimation vector size should equal ground truth vector size" << std::endl;
      return rmse;
  }

  for (int i=0; i < estimations.size(); ++i) {      
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse/estimations.size();
  rmse = sqrt(rmse.array());
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3,4);

  float p_x = x_state(0);
  float p_y = x_state(1);
  float v_x = x_state(2);
  float v_y = x_state(3);

  float c1 = p_x * p_x + p_y * p_y;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;
  
  Hj << (p_x/c2), (p_y/c2), 0, 0,
        -(p_y/c1), (p_x/c1), 0, 0,
        p_y*(v_x*p_y - v_y*p_x)/c3, p_x*(p_x*v_y - p_y*v_x)/c3, p_x/c2, p_y/c2;
  
  return Hj;
}

VectorXd Tools::ConvertCartesianToPolar(const VectorXd &x_state) {
    float p_x = x_state[0];
    float p_y = x_state[1];
    float v_x = x_state[2];
    float v_y = x_state[3];
    
    float rho = sqrt(p_x * p_x + p_y * p_y);
    float phi = atan2(p_y, p_x);
    float rho_dot = (p_x * v_x + p_y * v_y) / rho;
    
    VectorXd z = VectorXd(3);
    z << rho, phi, rho_dot;
    
    return z;
}
