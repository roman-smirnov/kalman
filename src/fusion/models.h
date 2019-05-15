#ifndef MODELS_H
#define MODELS_H

#include <utility>

namespace kalman {

struct LaserMeasurement final {
  long long timestamp = 0;  // epoch microseconds
  double x = 0;
  double y = 0;
};

struct RadarMeasurement final {
  long long timestamp = 0;  // epoch microseconds
  double rho = 0;
  double phi = 0;
  double rhodot = 0;
};

// state ground truth
struct Truth final {
  double x_truth = 0;
  double y_truth = 0;
  double vx_truth = 0;
  double vy_truth = 0;
};

// state prediction
struct Prediction final {
  double x = 0;
  double y = 0;
  double vx = 0;
  double vy = 0;
};

// root mean squared error (RMSE) filter evaluation metric
struct Evaluation final {
  double x_rmse = 0;
  double y_rmse = 0;
  double vx_rmse = 0;
  double vy_rmse = 0;
};

using Estimation = std::pair<Prediction, Evaluation>;

}

#endif //MODELS_H
