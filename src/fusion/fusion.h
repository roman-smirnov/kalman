#ifndef FUSION_H
#define FUSION_H

#include "models.h"
#include <memory>

namespace kalman {

class Fusion {

 public:
  virtual ~Fusion() = default;

  // process a radar measurement
  virtual Estimation ProcessMeasurement(RadarMeasurement &measurement, Truth &truth) = 0;

  // process a lidar measurement
  virtual Estimation ProcessMeasurement(LaserMeasurement &measurement, Truth &truth) = 0;


  // unscented filter factory method
  static std::unique_ptr<Fusion> GetInstance();
};

}

#endif //FUSION_H
