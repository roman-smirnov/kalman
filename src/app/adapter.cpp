#include "adapter.h"
#include "json.hpp"
#include <memory>
#include <string>
#include <istream>

namespace kalman {

class AdapterImpl final : public Adapter {
 public:
  void MessageToMeasurement(std::string message) override;
  std::string EstimationToMessage(Estimation estimation) override;
  void RegisterLaserHandler(std::function<void(LaserMeasurement, Truth)> LaserHandler) override;
  void RegisterRadarHandler(std::function<void(RadarMeasurement, Truth)> RadarHandler) override;

 private:
  static constexpr auto TELEMETRY_KEY = "telemetry";
  static constexpr auto MEASUREMENT_KEY = "sensor_measurement";
  static constexpr auto LASER_MEASUREMENT = "L";
  static constexpr auto RADAR_MEASUREMENT = "R";
  LaserMeasurement laser_measurement;
  RadarMeasurement radar_measurement;
  Truth truth;

  std::function<void(LaserMeasurement, Truth)> OnLaser;
  std::function<void(RadarMeasurement, Truth)> OnRadar;
};

void AdapterImpl::MessageToMeasurement(std::string message){
  // convert to json object
  auto j = nlohmann::json::parse(message);
  // validate event is telemetry
  if (j[0] != TELEMETRY_KEY)
    return;
  // extract raw measurement string
  auto raw_measurement = j[1].at(MEASUREMENT_KEY).get<std::string>();

  std::istringstream iss(raw_measurement);
  std::string sensor_type;
  iss >> sensor_type;
  if (sensor_type == LASER_MEASUREMENT) {
    iss >> laser_measurement.x;
    iss >> laser_measurement.y;
    iss >> laser_measurement.timestamp;
    iss >> truth.x_truth;
    iss >> truth.y_truth;
    iss >> truth.vx_truth;
    iss >> truth.vy_truth;
    if (OnLaser) this->OnLaser(laser_measurement, truth);
  } else if(sensor_type == RADAR_MEASUREMENT) {
    iss >> radar_measurement.rho;
    iss >> radar_measurement.phi;
    iss >> radar_measurement.rhodot;
    iss >> radar_measurement.timestamp;
    iss >> truth.x_truth;
    iss >> truth.y_truth;
    iss >> truth.vx_truth;
    iss >> truth.vy_truth;
    if (OnRadar) this->OnRadar(radar_measurement, truth);
  }
}

std::string AdapterImpl::EstimationToMessage(Estimation estimation) {
  nlohmann::json msgJson;
  msgJson["estimate_x"] = estimation.first.x;
  msgJson["estimate_y"] = estimation.first.y;
  msgJson["rmse_x"] = estimation.second.x_rmse;
  msgJson["rmse_y"] = estimation.second.y_rmse;
  msgJson["rmse_vx"] = estimation.second.vx_rmse;
  msgJson["rmse_vy"] = estimation.second.vy_rmse;
  auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
  return msg;
}

void AdapterImpl::RegisterLaserHandler(std::function<void(LaserMeasurement, Truth)> LaserHandler) {
  if (LaserHandler) this->OnLaser= LaserHandler;
}
void AdapterImpl::RegisterRadarHandler(std::function<void(RadarMeasurement, Truth)> RadarHandler) {
  if(RadarHandler) this->OnRadar = RadarHandler;
}

std::unique_ptr<Adapter> Adapter::GetInstance() {
  return std::make_unique<AdapterImpl>();
}

}


