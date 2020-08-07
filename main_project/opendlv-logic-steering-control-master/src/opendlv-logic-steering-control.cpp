#include <chrono>
#include <iostream>
#include <string>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("kf")) ||
      (0 == commandlineArguments.count("kn")) ||
      (0 == commandlineArguments.count("ki"))) {
    std::cerr << argv[0] << " controls a kiwi car "
              << "using a two-point model of steering." << std::endl
              << "Usage:   " << argv[0] << " --cid=<CID>"
              << " --kf=<constant front> --kn=<constant near>"
              << " --ki=<constant integral> [--verbose]"
              << "[--timemod=<Time scale modifier for simulation speed. "
                 "Default: 1.0>] "
              << std::endl
              << "Example: " << argv[0]
              << " --cid=111 --kf=1.0 --kn=1.0 --ki=1.0" << std::endl;
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
  float currentNearAngle{0.0f}, lastNearAngle{0.0f};
  float currentFarAngle{0.0f}, lastFarAngle{0.0f};
  float const deg2rad = 0.0174532925f;
  float const kf = deg2rad * std::stof(commandlineArguments["kf"]);
  float const kn = deg2rad * std::stof(commandlineArguments["kn"]);
  float const ki = deg2rad * std::stof(commandlineArguments["ki"]);
  float currentSteeringAngle{0.0f};
  cluon::data::TimeStamp lastTime{cluon::time::now()};

  auto updateSteeringAngle{[&od4, &currentNearAngle, &lastNearAngle,
                            &currentFarAngle, &lastFarAngle,
                            &currentSteeringAngle, &requestMutex, &kf, &kn, &ki,
                            &lastTime, &timemod, &verbose]() {
    {
      float const secondsToMicroseconds{1000 * 1000};
      std::lock_guard<std::mutex> lock(requestMutex);

      float changeNear{currentNearAngle - lastNearAngle};
      float changeFar{currentFarAngle - lastFarAngle};

      cluon::data::TimeStamp now = cluon::time::now();
      float dt{cluon::time::deltaInMicroseconds(now, lastTime) * timemod /
               secondsToMicroseconds};

      float changeSteeringAngle{kf * changeFar + kn * changeNear +
                                ki * currentNearAngle * dt};
      float newSteeringAngle{currentSteeringAngle - changeSteeringAngle};

      float const PI_HALF = 3.14f / 2.0f;
      newSteeringAngle = std::min(PI_HALF, newSteeringAngle);
      newSteeringAngle = std::max(-PI_HALF, newSteeringAngle);

      opendlv::proxy::GroundSteeringRequest groundSteeringRequest;
      groundSteeringRequest.groundSteering(newSteeringAngle);

      od4.send(groundSteeringRequest, cluon::time::now(), 0);

      lastNearAngle = currentNearAngle;
      lastFarAngle = currentFarAngle;
      currentSteeringAngle = newSteeringAngle;
      lastTime = now;

      if (verbose) {
        std::cout << "Sending new steering angle " << newSteeringAngle
                  << std::endl;
      }
    }
  }};

  auto onAimDirection{[&currentNearAngle, &lastNearAngle, &currentFarAngle,
                       &lastFarAngle, &requestMutex, &verbose,
                       &updateSteeringAngle](cluon::data::Envelope &&envelope) {
    uint32_t const senderStamp = envelope.senderStamp();
    auto msg = cluon::extractMessage<opendlv::logic::action::AimDirection>(
        std::move(envelope));
    {
      std::lock_guard<std::mutex> lock(requestMutex);
      if (senderStamp == 0) {
        currentNearAngle = msg.azimuthAngle();
      } else if (senderStamp == 1) {
        currentFarAngle = msg.azimuthAngle();
      }
    }
    if (verbose) {
      std::cout << "Got new " << (senderStamp == 0 ? "near" : "far") << "point "
                << msg.azimuthAngle() << std::endl;
    }
    updateSteeringAngle();
  }};

  od4.dataTrigger(opendlv::logic::action::AimDirection::ID(), onAimDirection);

  while (od4.isRunning()) {
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
  }

  return 0;
}
