#ifndef _MEASUREMENT_PACKAGE_H_
#define _MEASUREMENT_PACKAGE_H_

#include <istream>
#include <string>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  VectorXd raw_measurements_;
  VectorXd ground_truth_;
};


class MeasurementParser {
  public:
  static MeasurementPackage parse(std::istringstream &meas) {
    MeasurementPackage meas_package;
    long long timestamp;

    // reads first element from the current line
    std::string sensor_type;
    meas >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      meas >> px;
      meas >> py;
      meas_package.raw_measurements_ << px, py;
      meas >> timestamp;
      meas_package.timestamp_ = timestamp;
    } else if (sensor_type.compare("R") == 0) {
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      meas >> ro;
      meas >> theta;
      meas >> ro_dot;
      meas_package.raw_measurements_ << ro,theta, ro_dot;
      meas >> timestamp;
      meas_package.timestamp_ = timestamp;
    }

    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    meas >> x_gt;
    meas >> y_gt;
    meas >> vx_gt;
    meas >> vy_gt;

    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt; 
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;

    meas_package.ground_truth_ = gt_values;

    return meas_package;
  }
};

#endif // _MEASUREMENT_PACKAGE_H_