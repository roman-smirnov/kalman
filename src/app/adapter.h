#ifndef ADAPTER_H
#define ADAPTER_H

#include <models.h>
#include <functional>
#include <memory>
#include <iosfwd>

namespace kalman {

class Adapter {
 public:

  virtual ~Adapter() = default;

  virtual void RegisterLaserHandler(std::function<void(LaserMeasurement)> LaserHandler) = 0;
  virtual void RegisterRadarHandler(std::function<void(RadarMeasurement)> RadarHandler) = 0;

  virtual void MessageToMeasurement(std::string message) = 0;
  virtual std::string EstimationToMessage(Estimation estimation) = 0;

  // static factory method
  static std::unique_ptr<Adapter> GetInstance();
};

}

#endif //ADAPTER_H


