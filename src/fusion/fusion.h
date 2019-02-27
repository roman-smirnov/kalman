#ifndef FUSION_H
#define FUSION_H

#include "models.h"
#include <memory>

namespace kalman {

class Fusion {
 public:

  virtual ~Fusion() = default;

  virtual Estimation ProcessMeasurement(RadarMeasurement &measurement) = 0;

  virtual Estimation ProcessMeasurement(LaserMeasurement &measurement) = 0;

  // static factory method
  static std::unique_ptr<Fusion> GetInstance();

};



}

#endif //FUSION_H
