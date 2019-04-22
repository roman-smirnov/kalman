#include "adapter.h"
#include "json.hpp"

namespace kalman {

class AdapterImpl final : public Adapter {
 public:
  void MessageToMeasurement(std::string message) override;
  std::string EstimationToMessage(Estimation estimation) override;
  void RegisterLaserHandler(std::function<void(LaserMeasurement)> LaserHandler) override;
  void RegisterRadarHandler(std::function<void(RadarMeasurement)> RadarHandler) override;
 private:
  static constexpr auto TELEMETRY_KEY = "telemetry";
  static constexpr auto MEASUREMENT_KEY = "sensor_measurement";
  static constexpr auto LASER_MEASUREMENT = "L";
  static constexpr auto RADAR_MEASUREMENT = "R";

  LaserMeasurement laser_measurement;
  RadarMeasurement radar_measurement;

  std::function<void(LaserMeasurement)> OnLaser;
  std::function<void(RadarMeasurement)> OnRadar;

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
    iss >> laser_measurement.x_truth;
    iss >> laser_measurement.y_truth;
    iss >> laser_measurement.vx_truth;
    iss >> laser_measurement.vy_truth;
    if(OnLaser) this->OnLaser(laser_measurement);
  } else if(sensor_type == RADAR_MEASUREMENT) {
    iss >> radar_measurement.rho;
    iss >> radar_measurement.phi;
    iss >> radar_measurement.rhodot;
    iss >> radar_measurement.timestamp;
    iss >> radar_measurement.x_truth;
    iss >> radar_measurement.y_truth;
    iss >> radar_measurement.vx_truth;
    iss >> radar_measurement.vy_truth;
    if (OnRadar) this->OnRadar(radar_measurement);
  }
}

std::string AdapterImpl::EstimationToMessage(Estimation estimation) {
  nlohmann::json msgJson;
  msgJson["estimate_x"] = estimation.x;
  msgJson["estimate_y"] = estimation.y;
  msgJson["rmse_x"] = estimation.x_rmse;
  msgJson["rmse_y"] = estimation.y_rmse;
  msgJson["rmse_vx"] = estimation.vx_rmse;
  msgJson["rmse_vy"] = estimation.vy_rmse;
  auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
  return msg;
}

void AdapterImpl::RegisterLaserHandler(std::function<void(LaserMeasurement)> LaserHandler) {
  if (LaserHandler) this->OnLaser= LaserHandler;
}
void AdapterImpl::RegisterRadarHandler(std::function<void(RadarMeasurement)> RadarHandler) {
  if(RadarHandler) this->OnRadar = RadarHandler;
}

std::unique_ptr<Adapter> Adapter::GetInstance() {
  return std::unique_ptr<Adapter>(new AdapterImpl());
}

}


