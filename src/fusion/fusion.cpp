#include "fusion.h"
#include "laser_filter.cpp"
#include "radar_filter.cpp"
#include "evaluation.cpp"
#include <Eigen/Dense>
#include <cmath>

namespace kalman {

class FusionImpl final : public Fusion {

 public:

  Estimation ProcessMeasurement(LaserMeasurement &measurement) override;
  Estimation ProcessMeasurement(RadarMeasurement &measurement) override;

 private:

  void Predict(long long timestamp);

  void UpdateEstimation(Measurement &measurement);

  long long previous_timestamp = 0;

  static constexpr double acceleration_noise = 9.0;

  // state vector
  Eigen::Vector4d x = Eigen::Vector4d::Zero();

  // state transition matrix
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity()+Eigen::Matrix<double,6,6>::Identity().block<4,4>(2,0);

  // state covariance matrix
  Eigen::Matrix4d P = Eigen::Vector4d(1.0,1.0,1000.0,1000.0).asDiagonal();

  // process covariance matrix
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();

  Evaluation evaluation;
  Estimation estimation;

  LaserFilter laserFilter = LaserFilter(x,P);
  RadarFilter radarFilter = RadarFilter(x,P);

};


void FusionImpl::Predict(long long timestamp) {
  // compute timestamp microseconds difference, convert to seconds
  double dt = (timestamp - previous_timestamp) / 1000000.0;
  previous_timestamp = timestamp;
  // integrate time differences into state transition matrix F
  F.topRightCorner<2,2>() = Eigen::Matrix2d::Identity() * dt;

  // pre-compute process covariance matrix coefficients
  double dt_2 = std::pow(dt,2) * acceleration_noise;
  double dt_3 = std::pow(dt,3) * acceleration_noise/2.0;
  double dt_4 = std::pow(dt,4) * acceleration_noise/4.0;

  // compute the process covariance matrix Q
  Q << dt_4, 0, dt_3, 0,
       0, dt_4, 0, dt_3,
      dt_3, 0, dt_2, 0,
      0, dt_3, 0, dt_2;

  // compute new state
  x = F*x;
  // compute new state uncertainty matrix
  P = F * P * F.transpose() + Q;
}

Estimation FusionImpl::ProcessMeasurement(LaserMeasurement &measurement) {
  if (previous_timestamp == 0) {
    x << measurement.x, measurement.y, 0, 0;  // x, y, vx, vy
    previous_timestamp = measurement.timestamp;
    UpdateEstimation(measurement);
    return estimation;
  }
  Predict(measurement.timestamp);
  Eigen::Vector2d z(measurement.x, measurement.y);
  laserFilter.Update(z);
  UpdateEstimation(measurement);
  return estimation;
}

Estimation FusionImpl::ProcessMeasurement(RadarMeasurement &measurement) {
  if(previous_timestamp == 0){
    x << measurement.rho * std::cos(measurement.phi), measurement.rho*std::sin(measurement.phi), 0, 0;  // x, y, vx, vy
    previous_timestamp = measurement.timestamp;
    UpdateEstimation(measurement);
    return estimation;
  }
  Predict(measurement.timestamp);
  Eigen::Vector3d z(measurement.rho, measurement.phi, measurement.rhodot);
  radarFilter.Update(z);
  UpdateEstimation(measurement);
  return estimation;
}

void FusionImpl::UpdateEstimation(Measurement &measurement) {
  // calculate root mean squared errors
  Eigen::Array4d truth(measurement.x_truth, measurement.y_truth, measurement.vx_truth, measurement.vy_truth);
  Eigen::Array4d prediction = x.array();
  evaluation.Update(prediction, truth);
  auto rmse = evaluation.CalculateRMSE();
  // set estimation prediction values
  estimation.x = x[0];
  estimation.y = x[1];
  estimation.vx = x[2];
  estimation.vy = x[3];
  // set estimation rmse values
  estimation.x_rmse = rmse[0];
  estimation.y_rmse = rmse[1];
  estimation.vx_rmse = rmse[2];
  estimation.vy_rmse = rmse[3];
}

std::unique_ptr<Fusion> Fusion::GetInstance() {
  return std::unique_ptr<Fusion>(new FusionImpl());
}

}

