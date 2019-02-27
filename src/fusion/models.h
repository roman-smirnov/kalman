#ifndef MODELS_H
#define MODELS_H

namespace kalman {

struct Measurement {
  // epoch microseconds
  long long timestamp = 0;
  // state ground truth
  double x_truth = 0;
  double y_truth = 0;
  double vx_truth = 0;
  double vy_truth = 0;
};

struct LaserMeasurement final : Measurement {
  // laser sensor measurement values
  double x = 0;
  double y = 0;
};

struct RadarMeasurement final : Measurement {
  // radar sensor measurement values
  double rho = 0;
  double phi = 0;
  double rhodot = 0;
};

struct Estimation {
  // state prediction
  double x = 0;
  double y = 0;
  double vx = 0;
  double vy = 0;
  // root mean squared error
  double x_rmse = 0;
  double y_rmse = 0;
  double vx_rmse = 0;
  double vy_rmse = 0;
};

}

#endif //MODELS_H
