/*
================================================================================================================================

 network module

================================================================================================================================
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <string>
#include <functional>
#include <memory>


namespace kalman {


// handles simulator and logic networking via websockets
class Server {
 public:

  virtual ~Server() = default;

  // set callback to handle messages received from simulator
  virtual void RegisterOnReceived(std::function<void(std::string)> OnReceivedHandler) = 0;

  // start the server, begin handling requests and responses, etc
  virtual void Start() = 0;

  // send a message to the simulator
  virtual void Send(std::string msg) = 0;

  // static factory method
  static std::unique_ptr<Server> GetInstance();
};


}

#endif //NETWORK_SERVER_H
