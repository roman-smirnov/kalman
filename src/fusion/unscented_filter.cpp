//
// Created by Roman Smirnov on 2019-04-20.
//

#include "fusion.h"
#include "evaluation.cpp"
#include <Eigen/Dense>
#include <cmath>  // for std math functions: std::max, std::atan, etc.

namespace kalman {

class UnscentedFilter final : public Fusion {
 public:

  Estimation ProcessMeasurement(LaserMeasurement &measurement) override;

  Estimation ProcessMeasurement(RadarMeasurement &measurement) override;

 private:

  void Predict(long long timestamp);

  void UpdateEstimation(Measurement &measurement);

  // shorthand for an Mx1 column vector of double scalars
  template<int M>
  using VectorMd = Eigen::Matrix<double, M, 1>;

  // shorthand for an MxN dimensional matrix of double scalars
  template<int M, int N>
  using MatrixMNd = Eigen::Matrix<double, M, N>;

  // FIXME: nonesense names, no container
  // Process noise standard deviation longitudinal acceleration in m/s^2
  static constexpr double std_a = 30;
  // Process noise standard deviation yaw acceleration in rad/s^2
  static constexpr double std_yawdd = 30;
  // Laser measurement noise standard deviation position1 in m
  static constexpr double std_laspx = 0.15;    /// DO NOT MODIFY
  // Laser measurement noise standard deviation position2 in m
  static constexpr double std_laspy = 0.15;    /// DO NOT MODIFY
  // Radar measurement noise standard deviation radius in m
  static constexpr double std_radr = 0.3;    /// DO NOT MODIFY
  // Radar measurement noise standard deviation angle in rad
  static constexpr double std_radphi = 0.03;    /// DO NOT MODIFY
  // Radar measurement noise standard deviation radius change in m/s
  static constexpr double std_radrd = 0.3;    /// DO NOT MODIFY
  // number of state dimensions
  static constexpr int n_x = 5;
  // number of augmented state dimensions
  static constexpr int n_aug = 7;
  // number of augmented state sigma points
  static constexpr int n_sig = 2 * n_aug + 1;
  // Sigma point spreading parameter
  static constexpr double lambda = 3.0 - n_aug;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorMd<n_x> x = VectorMd<n_x>::Zero();

  // state covariance matrix
  MatrixMNd<n_x, n_x> P = MatrixMNd<n_x, n_x>::Identity();

  // declare and init weights of sigma points
  VectorMd<n_sig> weights =
      /// black magic sorcery
      (VectorMd<n_sig>() << lambda, VectorMd<n_sig - 1>::Constant(0.5)).finished() / (lambda + n_aug);

  // predicted sigma point state matrix
  MatrixMNd<n_x, n_sig> x_sig_pred = MatrixMNd<n_x, n_sig>::Zero();

  // augmented sigma point state matrix
  MatrixMNd<n_aug, n_sig> x_sig_aug = MatrixMNd<n_aug, n_sig>::Zero();

  // augmented state covariance vector
  VectorMd<n_aug> x_aug = VectorMd<n_aug>::Zero();

  // augmented state covariance matrix
  MatrixMNd<n_aug, n_aug> P_aug =
      /// demon summoning incantation
      (VectorMd<n_aug>() << VectorMd<n_x>::Ones(), VectorMd<2>(std_a,
                                                               std_yawdd).array().square()).finished().asDiagonal();

  // previous timestamp (UNIX Epoch time) in microseconds
  long long previous_timestamp = 0;

  // radar normalized innovation squared
  double nis_radar = 0.0;

  // laser normalized innovation squared
  double nis_laser = 0.0;

  // RMSE computed here
  Evaluation evaluation;
  Estimation estimation;
};

Estimation UnscentedFilter::ProcessMeasurement(LaserMeasurement &measurement) {
  if (previous_timestamp == 0) {
    // set initial state vector
    x << measurement.x, measurement.y, 0, 0, 0;
    // set initial timestamp
    previous_timestamp = measurement.timestamp;
    // set initial estimation equal to first measurement
    UpdateEstimation(measurement);
    return estimation;
  }
  Predict(measurement.timestamp);
  Eigen::Vector2d z(measurement.x, measurement.y);
  laserFilter.Update(z);
  UpdateEstimation(measurement);
  return estimation;
}

Estimation UnscentedFilter::ProcessMeasurement(RadarMeasurement &measurement) {
  if (previous_timestamp == 0) {
    // set initial state vector
    x << measurement.rho * std::cos(measurement.phi), measurement.rho * std::sin(measurement.phi), 0, 0, 0;
    // set initial timestamp
    previous_timestamp = measurement.timestamp;
    // set initial estimation equal to first measurement
    UpdateEstimation(measurement);
    return estimation;
  }
  UpdateEstimation(measurement);
  return estimation;
}

void UnscentedFilter::Predict(long long timestamp) {
  // microseconds to seconds,
  double dt = (timestamp - previous_timestamp) / 1000000.0;
  previous_timestamp = timestamp;

  // update augmented state and its covariance matrix
  x_aug.head(n_x) = x;
  P_aug.topLeftCorner<n_x, n_x>() = P;
  // compute matrix square root
  MatrixMNd<n_aug, n_aug> P_aug_sqrt = P_aug.llt().matrixL();
  // compute transformed sigma points
  x_sig_aug << x_aug, // FIXME: more witchcraft, also is slow
      x_aug.replicate<1, n_aug>() + std::sqrt(lambda + n_aug) * P_aug_sqrt,
      x_aug.replicate<1, n_aug>() - std::sqrt(lambda + n_aug) * P_aug_sqrt;

  if (c1 > 0.0001) {
    H << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return;
  }

}

void UnscentedFilter::UpdateEstimation(Measurement &measurement) {
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

std::unique_ptr<Fusion> Fusion::GetUkfInstance() {
  return std::make_unique<UnscentedFilter>();
}

}