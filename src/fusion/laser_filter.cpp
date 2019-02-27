//
// Created by Roman Smirnov on 2019-01-21.
//

#include <Eigen/Dense>

namespace kalman {

class LaserFilter final{

 public:

  explicit LaserFilter(Eigen::Vector4d &x, Eigen::Matrix4d &P): x(x), P(P) {}
  void Update(Eigen::Vector2d &z);

 private:

  // state vector
  Eigen::Vector4d &x;

  // state covariance matrix
  Eigen::Matrix4d &P;

  // measurement matrix
  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double,2,4>::Identity();

  // measurement covariance matrix
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity()*0.0225;

};

void LaserFilter::Update(Eigen::Vector2d &z) {
  auto y = z - H * x;;
  auto S = H * P * H.transpose() + R;
  auto K = P * H.transpose() * S.inverse();
  x = x+(K * y);
  P = (Eigen::Matrix4d::Identity() - K * H)*P;
}

}