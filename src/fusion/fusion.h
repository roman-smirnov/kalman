#ifndef FUSION_H
#define FUSION_H

#include "models.h"
#include <memory>

namespace kalman {

class Fusion {

 public:
  virtual ~Fusion() = default;

  // process a radar measurement
  virtual Estimation ProcessMeasurement(RadarMeasurement &measurement) = 0;

  // process a lidar measurement
  virtual Estimation ProcessMeasurement(LaserMeasurement &measurement) = 0;

  // extended filter factory method
  static std::unique_ptr<Fusion> GetEkfInstance();

  // unscented filter factory method
  static std::unique_ptr<Fusion> GetUkfInstance();
};

}

#endif //FUSION_H
