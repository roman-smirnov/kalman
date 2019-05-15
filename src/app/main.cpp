
#include "adapter.h"
#include <models.h>
#include <server.h>
#include <fusion.h>
#include <string>
#include <memory>

int main() {

  // init system components
  std::unique_ptr<kalman::Server> server = kalman::Server::GetInstance();
  std::unique_ptr<kalman::Adapter> adapter = kalman::Adapter::GetInstance();
  std::unique_ptr<kalman::Fusion> fusion = kalman::Fusion::GetEkfInstance();

  // handle laser measurement callback
  adapter->RegisterLaserHandler([&](kalman::LaserMeasurement measurement, kalman::Truth truth) {
    auto estimation = fusion->ProcessMeasurement(measurement, truth);
    auto msg = adapter->EstimationToMessage(estimation);
    server->Send(msg);
  });

  // handle radar measurement callback
  adapter->RegisterRadarHandler([&](kalman::RadarMeasurement measurement, kalman::Truth truth) {
    auto estimation = fusion->ProcessMeasurement(measurement, truth);
    auto msg = adapter->EstimationToMessage(estimation);
    server->Send(msg);
  });

  // handle server message callback
  server->RegisterOnReceived([&](std::string msg){
    adapter->MessageToMeasurement(msg);
  });

  // start server
  server->Start();

  return EXIT_SUCCESS;
}

