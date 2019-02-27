
#include <Eigen/Dense>
#include <cmath>

namespace kalman {

class RadarFilter final{

 public:

  RadarFilter(Eigen::Vector4d &x, Eigen::Matrix4d &P): x(x), P(P) {}
  void Update(Eigen::Vector3d &z);

 private:

  Eigen::Vector3d GetStateAsPolar();

  void UpdateMeasurementMatrix();

  // state vector
  Eigen::Vector4d &x;

  // state covariance matrix
  Eigen::Matrix4d &P;

  // measurement matrix
  Eigen::Matrix<double, 3, 4> H = Eigen::Matrix<double , 3, 4>::Zero();

  // measurement covariance matrix
  Eigen::Matrix3d R = Eigen::Vector3d(0.09,0.0009,0.09).asDiagonal();

  static constexpr double PI = 3.14159265358979323846;
  static constexpr double TWO_PI = PI * 2.0;

};

// compute and update Jacobian linearization measurement matrix (AKA Hj)
void RadarFilter::UpdateMeasurementMatrix() {
  // precompute matrix terms
  double c1 = std::pow(x[0], 2) + std::pow(x[1], 2);
  double c2 = std::sqrt(c1);
  double c3 = c1 * c2;
  //avoid division by zero
  if(c1 < 0.0001){
    H << 0,0,0,0,0,0,0,0,0,0,0,0;
    return;
  }
  //compute the Jacobian matrix
  H << x[0]/c2, x[1]/c2, 0, 0,
      -x[1]/c1, x[0]/c1, 0, 0,
      x[1] * (x[2] * x[1] - x[3] * x[0])/c3, x[0] * (x[0] * x[3] - x[1] * x[2])/c3, x[0] / c2, x[1] / c2;
}

Eigen::Vector3d RadarFilter::GetStateAsPolar(){
  // convert current state to polar
  double rho = std::max(std::sqrt(std::pow(x[0],2)+std::pow(x[1],2)),  0.000001);
  double phi = std::atan2(x[1], x[0]);  // returns values between -pi and pi
  double rho_dot = (x[0] * x[2] + x[1] * x[3])/rho; // avoid division by 0
  return Eigen::Vector3d(rho, phi, rho_dot);
}

void RadarFilter::Update(Eigen::Vector3d &z) {
  // compute Jacobian measurement linearization matrix (AKA Hj)
  UpdateMeasurementMatrix();

  // convert current cartesian state to polar representation
  auto z_pred = GetStateAsPolar();
  Eigen::Vector3d y = z - z_pred;
  y(1) = y(1) - TWO_PI * (int)(y(1) / TWO_PI); // normalize to [-pi, pi]

  // update state vector and uncertainty matrix
  auto S = H * P * H.transpose() + R;
  auto K = P * H.transpose() * S.inverse();
  x = x+(K * y);
  P = (Eigen::Matrix4d::Identity() - K * H)*P;
}

}
