/*
================================================================================================================================

 implementation of network module

================================================================================================================================
*/

#include "server.h"
#include <uWS/uWS.h>
#include <iostream>
#include <memory>

namespace kalman {

class ServerImpl final : public Server {

 public:

  void RegisterOnReceived(std::function<void(std::string)> OnReceivedHandler) override;

  void Start() override;

  void Send(std::string msg) override;

 private:

  // validate raw incoming message, extract payload and forward to handler callback
  std::string ValidateAndExtract(std::string message);

  // callback function to handle messages received from simulator
  std::function<void(std::string)> OnReceived;

  // set uWS websocket mechanism event/message/etc callback handlers
  void SetWebsocketHandlers();

  //  attach websocket mechanism to network port
  void BeginListening();

  // simulator client localhost port
  static constexpr int SIMULATOR_PORT_NUMBER = 4567;

  // various string constants
  static constexpr auto CONNECTED_MESSAGE = "Connected to simulator";
  static constexpr auto DISCONNECTED_MESSAGE = "Disconnected from simulator";
  static constexpr auto LISTENING_MESSAGE = "Listening on assigned port ";
  static constexpr auto LISTENING_ERROR_MESSAGE = "Failed to start listening on assigned port ";
  static constexpr auto RECEIVED_MESSAGE = "Received message from simulator";

  // websocket message prefix
  static constexpr char MESSAGE_CODE = '4';
  static constexpr char EVENT_CODE = '2';
  static constexpr size_t CODE_LENGTH = 2;

  // default server response to messages with no data
  static constexpr auto MANUAL_DRIVING = "42[\"manual\",{}]";

  // websocket low level handling and managment (uWebsockets library)
  uWS::Hub websocket_hub;

};

void ServerImpl::RegisterOnReceived(std::function<void(std::string)> OnReceivedHandler) {
  if (OnReceivedHandler) this->OnReceived = OnReceivedHandler;
}

void ServerImpl::SetWebsocketHandlers() {

  // client connected
  websocket_hub.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << CONNECTED_MESSAGE << std::endl;
  });

  // client disconnected
  websocket_hub.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    std::cout << DISCONNECTED_MESSAGE << std::endl;
  });

  // message received
  websocket_hub.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    auto message = this->ValidateAndExtract(std::string(data, length));
    if (!message.empty() && this->OnReceived)
      this->OnReceived(message);
  });
}

std::string ServerImpl::ValidateAndExtract(std::string message) {
  // message should start with websocket prefix code
  if (message.length() < CODE_LENGTH || message[0] != MESSAGE_CODE || message[1] != EVENT_CODE)
    return "";
  // should contain payload
  if (message.find("null") != std::string::npos) {
    this->Send("42[\"manual\",{}]");
    return "";
  }
  // should contain openning and closing square brackets
  auto b1 = message.find_first_of('[');
  auto b2 = message.find_first_of(']');
  if (b1 == std::string::npos || b2 == std::string::npos) {
    this->Send("42[\"manual\",{}]");
    return "";
  }
  // extract json array payload
  return message.substr(b1, b2 - b1 + 1);
}

void ServerImpl::Start() {
  SetWebsocketHandlers();
  BeginListening();
  websocket_hub.run();
}

void ServerImpl::Send(std::string msg) {
  websocket_hub.getDefaultGroup<uWS::SERVER>().broadcast(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void ServerImpl::BeginListening() {
  if (websocket_hub.listen(SIMULATOR_PORT_NUMBER)) {
    std::cout << LISTENING_MESSAGE << SIMULATOR_PORT_NUMBER << std::endl;
  } else {
    std::cerr << LISTENING_ERROR_MESSAGE << SIMULATOR_PORT_NUMBER << std::endl;
    exit(EXIT_FAILURE);
  }
}

std::unique_ptr<Server> Server::GetInstance() {
  return std::make_unique<ServerImpl>();
}

}