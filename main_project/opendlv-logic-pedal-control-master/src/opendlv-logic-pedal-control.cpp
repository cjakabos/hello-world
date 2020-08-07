#include <chrono>
#include <iostream>
#include <string>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("mxp")) ||
      (0 == commandlineArguments.count("mip"))) {
    std::cerr << argv[0] << " Control speed of a kiwi car " << std::endl
              << "Usage:   " << argv[0] << " --cid=<CID>"
              << " --mxp=<constant maximum pedalposition>"
              << " --mip=<constant minimum pedalposition> [--verbose]"
              << "[--timemod=<Time scale modifier for simulation speed. "
                 "Default: 1.0>] "
              << std::endl
              << "Example: " << argv[0]
              << " --cid=111 --mxp=0.1 --mip=0.01" << std::endl;
    return 1;
  }

  bool const verbose{commandlineArguments.count("verbose") != 0};
  float const timemod{
      (commandlineArguments["timemod"].size() != 0)
          ? static_cast<float>(std::stof(commandlineArguments["timemod"]))
          : 1.0f};

  cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

  std::mutex requestMutex;
  float const mxp = std::stof(commandlineArguments["mxp"]);
  float const mip = std::stof(commandlineArguments["mip"]);
  float groundSteering{0.0f};
  float pedalPosition{0.01f};
  float kiwiDistanceX{0.0f};
  float kiwiDistanceY{0.0f};
  bool intersection{false};

  auto updatePedalPosition{[&od4, &groundSteering, &pedalPosition, 
                            &intersection, &kiwiDistanceY, &kiwiDistanceX,
                            &requestMutex, &mxp, &mip, &timemod, &verbose]() {
    {
      std::lock_guard<std::mutex> lock(requestMutex);

      if(intersection && (kiwiDistanceX > -400) && 
        (kiwiDistanceX < 400) && (kiwiDistanceY < 635)){
        pedalPosition = 0.0;   
      }
      else if((kiwiDistanceX > -600) && (kiwiDistanceX < 600)){
        if(kiwiDistanceY < 1000){
          float KP = 1.0f/1500.0f; 
          float desiredDistance = 600;
          pedalPosition = (kiwiDistanceY-desiredDistance)*KP;
          pedalPosition = std::min(mxp, pedalPosition);
          pedalPosition = std::max(0.0f, pedalPosition);
        }
      }
      else{
        if(std::abs(groundSteering) < 0.1f) {
          pedalPosition = mxp;
        } 
        else {
          float const PI_QUARTER = 3.14f / 4.0f;
          pedalPosition = mip + (mxp-mip)*(1.0f - 
                               std::abs(groundSteering)/PI_QUARTER);
          pedalPosition = std::min(mxp, pedalPosition);
          pedalPosition = std::max(mip, pedalPosition);
        }
      }

      opendlv::proxy::PedalPositionRequest pedalPositionRequest;
      pedalPositionRequest.position(pedalPosition);

      od4.send(pedalPositionRequest, cluon::time::now(), 0);

      if (verbose) {
        std::cout << "Sending new pedal position " << pedalPosition
                  << std::endl;
      }
    }
  }};

  auto onGroundSteeringRequest{[&requestMutex, &verbose,
                       &updatePedalPosition, &groundSteering]
                       (cluon::data::Envelope &&envelope) {
    uint32_t const senderStamp = envelope.senderStamp();
    auto msg = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(
        std::move(envelope));
    {
      std::lock_guard<std::mutex> lock(requestMutex);
      if (senderStamp == 0) {
        groundSteering = msg.groundSteering();
      }
    }
    if (verbose) {
      std::cout << "Got new steering angle "
                << msg.groundSteering() << std::endl;
    }
    updatePedalPosition();
  }};

  auto onObjectDistance{[&requestMutex, &kiwiDistanceX, &kiwiDistanceY,
                         &intersection, &verbose, &updatePedalPosition]
                         (cluon::data::Envelope &&envelope){
    uint32_t const senderStamp = envelope.senderStamp();
    auto msg = cluon::extractMessage<opendlv::logic::perception::ObjectDistance>(
        std::move(envelope));
    {
      std::lock_guard<std::mutex> lock(requestMutex);
      if (senderStamp == 0) {
        kiwiDistanceY = msg.distance();
      }
      if (senderStamp == 1) {
        kiwiDistanceX = msg.distance();
      }
    }
    if (verbose) {
      std::cout << "Got new "<< (senderStamp == 0 ? "Y" : "X") <<
      " distance to kiwi " << msg.distance() << std::endl;
    }
   }};


  auto onObject{[&requestMutex, &intersection, &verbose]
                (cluon::data::Envelope &&envelope){
    uint32_t const senderStamp = envelope.senderStamp();
    auto msg = cluon::extractMessage<opendlv::logic::perception::Object>(
        std::move(envelope));
    {
      std::lock_guard<std::mutex> lock(requestMutex);
      if (senderStamp == 1) {
        if (msg.objectId() == 1){
          intersection = true;
        }
        else{
          intersection = false;
        }
      }
    }
    if (verbose) {
      std::cout << "Intersection " << intersection << std::endl;
    }
   }};

  od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), 
  onGroundSteeringRequest);
  od4.dataTrigger(opendlv::logic::perception::Object::ID(), onObject);
  od4.dataTrigger(opendlv::logic::perception::ObjectDistance::ID(),
  onObjectDistance);

  while (od4.isRunning()) {
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  }

  return 0;
}
