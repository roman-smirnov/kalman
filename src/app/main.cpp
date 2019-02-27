
#include "adapter.h"
#include <models.h>
#include <server.h>
#include <fusion.h>
#include <string>


int main() {

  // init system components
  auto server = kalman::Server::GetInstance();
  auto adapter = kalman::Adapter::GetInstance();
  auto fusion = kalman::Fusion::GetInstance();

  // handle laser measurement callback
  adapter -> RegisterLaserHandler([&](kalman::LaserMeasurement measurement){
    auto estimation = fusion->ProcessMeasurement(measurement);
    auto msg = adapter->EstimationToMessage(estimation);
    server->Send(msg);
  });

  // handle radar measurement callback
  adapter -> RegisterRadarHandler([&](kalman::RadarMeasurement measurement){
    auto estimation = fusion->ProcessMeasurement(measurement);
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
