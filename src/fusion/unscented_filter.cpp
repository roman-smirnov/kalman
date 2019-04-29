#include "fusion.h"
#include "models.h"
#include "evaluation.cpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace kalman {

namespace {
// TODO: convert all constant to use `k` prefix and PascalCase
// Process noise standard deviation longitudinal acceleration in m/s^2
static constexpr double std_a = 0.9;  /// DO OPTIMIZE THIS
// Process noise standard deviation yaw acceleration in rad/s^2
static constexpr double std_yawdd = 0.6;  /// DO OPTIMIZE THIS
// Laser measurement noise standard deviation position1 in m
static constexpr double std_laspx = 0.15;  /// DO NOT MODIFY
// Laser measurement noise standard deviation position2 in m
static constexpr double std_laspy = 0.15;  /// DO NOT MODIFY
// Radar measurement noise standard deviation radius in m
static constexpr double std_radr = 0.3;  /// DO NOT MODIFY
// Radar measurement noise standard deviation angle in rad
static constexpr double std_radphi = 0.03;  /// DO NOT MODIFY
// Radar measurement noise standard deviation radius change in m/s
static constexpr double std_radrd = 0.3;  /// DO NOT MODIFY
// number of lidar measurement dimensions
static constexpr int n_lidar = 2;
// number of radar measurement dimensions
static constexpr int n_radar = 3;
// number of state dimensions
static constexpr int n_x = 5;
// number of augmented state dimensions
static constexpr int n_aug = 7;
// number of augmented state sigma points
static constexpr int n_sig = 2 * n_aug + 1;
// sigma point spreading parameter
static constexpr double kLambda = 3.0 - n_aug;
// machine epsilon of type double (IMPORTANT: not smallest possible positive double)
static constexpr double epsilon = std::numeric_limits<double>::epsilon();
// i.e compile time double constant version of M_PI
static constexpr double kPi = 3.1415926535897932385L;
// i.e 2*M_PI, also known as Tau
static constexpr double kTwoPi = 2.0 * kPi;
// multiply to convert seconds to micro-seconds
static constexpr double kSecToMicrosecFactor = 1.0e-6;

// shorthand for an Mx1 column vector of double scalars
template<int M>
using Vector = Eigen::Matrix<double, M, 1>;


// shorthand for an MxN dimensional matrix of double scalars
template<int M, int N = M>
using Matrix = Eigen::Matrix<double, M, N>;

// very dark magic
template<int M, int N = M>
constexpr auto MakeMatrix = [](auto m, auto... ms) { return ((Matrix<M, N>() << m), ..., ms).finished(); };

template<int M, int N = M>
constexpr auto GetRows = [](Matrix<M, N> m) { return std::make_tuple(...) };

constexpr auto Square = [](double x) { return std::pow(x, 2); };

constexpr auto Sqrt = [](double x) { return std::sqrt(x); };

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
  Matrix<n_radar, n_sig> TransformSigmaPointsToRadar();
  Matrix<n_lidar, n_sig> TransformSigmaPointsToLidar();
  //internal state update
  void UpdateState();
  void UpdateEstimation(Measurement &measurement);
  //TODO: static GetRows<>() func
  //TODO: static Mean<>() and Covariance<>() funcs

 private:
  // previous timestamp (UNIX Epoch time) in microseconds
  long long previous_timestamp = 0;

  // radar normalized innovation squared
  double nis_radar = 0.0;

  // laser normalized innovation squared
  double nis_laser = 0.0;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Vector<n_x> x = Vector<n_x>::Zero();

  // state covariance matrix
  Matrix<n_x> P = Matrix<n_x>::Identity();

  // predicted sigma point state matrix
  Matrix<n_x, n_sig> x_sig_pred = Matrix<n_x, n_sig>::Zero();

  // augmented sigma point state matrix
  Matrix<n_aug, n_sig> x_sig_aug = Matrix<n_aug, n_sig>::Zero();

  // weights of sigma points
  const Vector<n_sig> weights =  // black magic sorcery
      (Vector<n_sig>() << kLambda, Vector<n_sig - 1>::Constant(0.5)).finished() / (kLambda + n_aug);

  // radar measurement covariance matrix
  const Matrix<n_radar> R_radar =
      Vector<n_radar>(Square(std_radr), Square(std_radphi), Square(std_radrd)).asDiagonal();

  const Matrix<n_lidar> R_lidar =
      Vector<n_lidar>(Square(std_laspx), Square(std_laspy)).asDiagonal();

  // process noise covariance matrix
  const Matrix<2> Q = Vector<2>(Square(std_a), Square(std_yawdd)).asDiagonal();

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
  Vector<n_aug> x_aug = Matrix<n_aug, n_x>::Identity() * x;
  // compute augmented state process covariance
  Matrix<n_aug, n_aug> P_aug = Matrix<n_aug, n_x>::Identity() * P * Matrix<n_x, n_aug>::Identity();
  P_aug.bottomRightCorner<2, 2>() = Q;  // Q isn't used anywhere else, better hold as a 7x7 matrix from get-go
  // compute matrix square root
  Matrix<n_aug, n_aug> P_aug_sqrt = P_aug.llt().matrixL();
  // compute transformed sigma points
  x_sig_aug = x_aug.replicate<1, n_sig>();
  x_sig_aug.block<0, 1>(n_aug, n_aug) += std::sqrt(kLambda + n_aug) * P_aug_sqrt;
  x_sig_aug.block<0, n_aug + 1>(n_aug, n_aug) -= std::sqrt(kLambda + n_aug) * P_aug_sqrt;
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
  yawd += 2.0 * epsilon * (yawd.abs() < epsilon).cast<double>();
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
  Matrix<n_x, n_sig> pred_sig_dif = x_sig_pred - x.replicate<1, n_sig>();
  // normalize to -pi,pi
  pred_sig_dif.row(3).array() -= (((1.0 / kPi) * pred_sig_dif.row(3).array()).floor()) * kPi;
  P << pred_sig_dif * weights.asDiagonal() * pred_sig_dif.transpose();
}

Matrix<n_radar, n_sig> UnscentedFilter::RadarSigmaPoints() {
  auto px = x_sig_pred.row(0).array();
  auto py = x_sig_pred.row(1).array();
  auto v = x_sig_pred.row(2).array();
  auto yaw = x_sig_pred.row(3).array();
  auto yawd = x_sig_pred.row(4).array();
  // avoid division by zero
  px = (px.abs() > epsilon).select(px, epsilon);
  py = (py.abs() > epsilon).select(py, epsilon);
  // transform sigma points to measurement space
  auto rho = (px.square() + py.square()).sqrt();
  auto phi = px.binaryExpr(py, [](double x, double y) { return std::atan2(y, x); });
  auto rho_dot = (px * yaw.cos() + py * yaw.sin()) * v / rho;
  // predicted sigma points in measurement space
  return (Matrix<n_radar, n_sig>() << rho, phi, rho_dot).finished();
}

void UnscentedFilter::UpdateState() {
  auto z_sig = RadarSigmaPoints();
  // measurement mean
  Vector<n_radar> z_pred = z_sig * weights;
  // measurement covariance
  auto y_z = z_sig - z_pred.replicate<1, n_sig>();
  Matrix<n_radar> S = y_z * weights.asDiagonal() * y_z.transpose() + R_radar;

  auto y_x = x_sig_pred - x.replicate<1, n_sig>();
  // cross correlation between predicted points in state and measurement space
  Matrix<n_radar> T = y_x * weights.asDiagonal() * y_z.transpose();
  // kalman gain
  Matrix<n_radar> K = T * S.inverse();

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

std::unique_ptr<Fusion> Fusion::GetUkfInstance() {
  return std::make_unique<UnscentedFilter>();
}

}