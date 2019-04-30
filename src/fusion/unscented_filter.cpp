#include "fusion.h"
#include "models.h"
#include "evaluation.cpp"
#include "common.h"

namespace kalman::fusion::internal {

namespace {
// Process noise standard deviation longitudinal acceleration in m/s^2
constexpr double kAccelNoise = 0.9;  /// DO OPTIMIZE THIS
// Process noise standard deviation yaw acceleration in rad/s^2
constexpr double kYawdNoise = 0.6;  /// DO OPTIMIZE THIS
// Laser measurement noise standard deviation position1 in m
constexpr double kLidarMeasPxNoise = 0.15;  /// DO NOT MODIFY
// Laser measurement noise standard deviation position2 in m
constexpr double kLidarMeasPyNoise = 0.15;  /// DO NOT MODIFY
// Radar measurement noise standard deviation radius in m
constexpr double kRadarMeasRhoNoise = 0.3;  /// DO NOT MODIFY
// Radar measurement noise standard deviation angle in rad
constexpr double kRadarMeasPhiNoise = 0.03;  /// DO NOT MODIFY
// Radar measurement noise standard deviation radius change in m/s
constexpr double kRadarMeasRhodotNoise = 0.3;  /// DO NOT MODIFY
// number of lidar measurement dimensions
constexpr int kDimLidar = 2;
// number of radar measurement dimensions
constexpr int kDimRadar = 3;
// number of state dimensions
constexpr int kDimState = 5;
// number of augmented state dimensions
constexpr int kDimAugState = 7;
// number of augmented state sigma points
constexpr int kNumSigmaPts = 2 * kDimAugState + 1;
// sigma point spreading parameter
constexpr double kLambda = 3.0 - kDimAugState;
}//anonymous namespace


class UnscentedFilter final : public Fusion {
 public:
  Estimation ProcessMeasurement(LaserMeasurement &measurement) override;
  Estimation ProcessMeasurement(RadarMeasurement &measurement) override;

 private:
  // internal state prediction (measurement independent)
  void Predict(long long timestamp);
  void GenerateSigmaPoints();
  void TransformSigmaPoints(double dt);
  void ReduceSigmaPoints();
  // measurement space state prediction
  Matrix<kDimRadar, kNumSigmaPts> TransformSigmaPointsToRadar();
  Matrix<kDimLidar, kNumSigmaPts> TransformSigmaPointsToLidar();
  //internal state update
  void UpdateState();
  void UpdateEstimation(Measurement &measurement);
  //TODO: static Mean<>() and Covariance<>() funcs

 private:
  // previous timestamp (UNIX Epoch time) in microseconds
  long long previous_timestamp = 0;

  // radar normalized innovation squared
  double nis_radar = 0.0;

  // laser normalized innovation squared
  double nis_laser = 0.0;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Vector<kDimState> x = Vector<kDimState>::Zero();

  // state covariance matrix
  Matrix<kDimState> P = Matrix<kDimState>::Identity();

  // predicted sigma point state matrix
  Matrix<kDimState, kNumSigmaPts> x_sig_pred = Matrix<kDimState, kNumSigmaPts>::Zero();

  // augmented sigma point state matrix
  Matrix<kDimAugState, kNumSigmaPts> x_sig_aug = Matrix<kDimAugState, kNumSigmaPts>::Zero();

  // weights of sigma points
  const Vector<kNumSigmaPts> weights =  // black magic sorcery
      (Vector<kNumSigmaPts>() << kLambda, Vector<kNumSigmaPts - 1>::Constant(0.5)).finished()
          / (kLambda + kDimAugState);

  // radar measurement covariance matrix
  const Matrix<kDimRadar> R_radar =
      Vector<kDimRadar>(Square(kRadarMeasRhoNoise),
                        Square(kRadarMeasPhiNoise),
                        Square(kRadarMeasRhodotNoise)).asDiagonal();

  const Matrix<kDimLidar> R_lidar =
      Vector<kDimLidar>(Square(kLidarMeasPxNoise), Square(kLidarMeasPyNoise)).asDiagonal();

  // process noise covariance matrix
  const Matrix<2> Q = Vector<2>(Square(kAccelNoise), Square(kYawdNoise)).asDiagonal();

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
//  laserFilter.Update(z);
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
  double dt = kSecToMicrosecFactor * (timestamp - previous_timestamp);
  previous_timestamp = timestamp;
  GenerateSigmaPoints();
  TransformSigmaPoints(dt);
  ReduceSigmaPoints();
}

void UnscentedFilter::GenerateSigmaPoints() {
  // compute augmented state
  Vector<kDimAugState> x_aug = Matrix<kDimAugState, kDimState>::Identity() * x;
  // compute augmented state process covariance
  Matrix<kDimAugState, kDimAugState>
      P_aug = Matrix<kDimAugState, kDimState>::Identity() * P * Matrix<kDimState, kDimAugState>::Identity();
  P_aug.bottomRightCorner<2, 2>() = Q;  // Q isn't used anywhere else, better hold as a 7x7 matrix from get-go
  // compute matrix square root
  Matrix<kDimAugState, kDimAugState> P_aug_sqrt = P_aug.llt().matrixL();
  // compute transformed sigma points
  x_sig_aug = x_aug.replicate<1, kNumSigmaPts>();
  x_sig_aug.block<0, 1>(kDimAugState, kDimAugState) += std::sqrt(kLambda + kDimAugState) * P_aug_sqrt;
  x_sig_aug.block<0, kDimAugState + 1>(kDimAugState, kDimAugState) -= std::sqrt(kLambda + kDimAugState) * P_aug_sqrt;
}

void UnscentedFilter::TransformSigmaPoints(double dt) {
  // each row corresponds to an augmented state dimension
  auto pos_x = x_sig_aug.row(0).array();
  auto pos_y = x_sig_aug.row(1).array();
  auto vel = x_sig_aug.row(2).array();
  auto yaw = x_sig_aug.row(3).array();
  auto yawd = x_sig_aug.row(4).array();
  auto nu_acel = x_sig_aug.row(5).array();
  auto nu_yawdd = x_sig_aug.row(6).array();
  // add epsilon to all yawd which are approx equal to zero
  yawd += 2.0 * kEpsilon * (yawd.abs() < kEpsilon).cast<double>();
  // compute predicted sigma points
  auto px_pred = pos_x + (vel / yawd) * ((yaw + yawd * dt).sin() - yaw.sin()) + 0.5 * nu_acel * Square(dt) * yaw.cos();
  auto py_pred = pos_y + (vel / yawd) * (-(yaw + yawd * dt).cos() + yaw.cos()) + 0.5 * nu_acel * Square(dt) * yaw.sin();
  auto vel_pred = vel + nu_acel * dt;
  auto yaw_pred = yaw + yawd + 0.5 * nu_yawdd * Square(dt);
  auto yawd_pred = yawd + nu_yawdd * dt;
  // predicted sigma points
  x_sig_pred << px_pred, py_pred, vel_pred, yaw_pred, yawd_pred;
}

void UnscentedFilter::ReduceSigmaPoints() {
  // prediction mean state
  x << x_sig_pred * weights;
  // prediction process covariance
  Matrix<kDimState, kNumSigmaPts> pred_sig_dif = x_sig_pred - x.replicate<1, kNumSigmaPts>();
  // normalize to -pi,pi
  pred_sig_dif.row(3).array() -= (((1.0 / kPi) * pred_sig_dif.row(3).array()).floor()) * kPi;
  P << pred_sig_dif * weights.asDiagonal() * pred_sig_dif.transpose();
}

Matrix<kDimRadar, kNumSigmaPts> UnscentedFilter::RadarSigmaPoints() {
  auto px = x_sig_pred.row(0).array();
  auto py = x_sig_pred.row(1).array();
  auto v = x_sig_pred.row(2).array();
  auto yaw = x_sig_pred.row(3).array();
  auto yawd = x_sig_pred.row(4).array();
  // avoid division by zero
  px = (px.abs() > kEpsilon).select(px, kEpsilon);
  py = (py.abs() > kEpsilon).select(py, kEpsilon);
  // transform sigma points to measurement space
  auto rho = (px.square() + py.square()).sqrt();
  auto phi = px.binaryExpr(py, [](double x, double y) { return std::atan2(y, x); });
  auto rho_dot = (px * yaw.cos() + py * yaw.sin()) * v / rho;
  // predicted sigma points in measurement space
  return (Matrix<kDimRadar, kNumSigmaPts>() << rho, phi, rho_dot).finished();
}

void UnscentedFilter::UpdateState() {
  auto z_sig = RadarSigmaPoints();
  // measurement mean
  Vector<kDimRadar> z_pred = z_sig * weights;
  // measurement covariance
  auto y_z = z_sig - z_pred.replicate<1, kNumSigmaPts>();
  Matrix<kDimRadar> S = y_z * weights.asDiagonal() * y_z.transpose() + R_radar;

  auto y_x = x_sig_pred - x.replicate<1, kNumSigmaPts>();
  // cross correlation between predicted points in state and measurement space
  Matrix<kDimRadar> T = y_x * weights.asDiagonal() * y_z.transpose();
  // kalman gain
  Matrix<kDimRadar> K = T * S.inverse();

  // update state
  x += K * y_z;
}

void UnscentedFilter::UpdateEstimation(Measurement &measurement) {
  // calculate root mean squared errors
  Eigen::Array4d truth(measurement.x_truth, measurement.y_truth, measurement.vx_truth, measurement.vy_truth);
  Eigen::Array4d prediction = x.array();
  evaluation.Update(prediction, truth);
  auto rmse = evaluation.CalculateRMSE();
//  Estimation estimation;
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

//TODO: move this to somewhere in kalman scope
//std::unique_ptr<Fusion> Fusion::GetUkfInstance() {
//  return std::make_unique<UnscentedFilter>();
//}

}