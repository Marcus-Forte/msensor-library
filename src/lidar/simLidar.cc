#include "lidar/simLidar.hh"
#include <chrono>
#include <iostream>
#include <thread>

void SimLidar::init() { std::cout << "init" << std::endl; }

Scan2D SimLidar::getScan() {
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz.
  return {0, {{0.0, 0.0}, {1.0, 1.0}, {1, -1}, {-1, -1}, {-1, 1}}};
}

void SimLidar::setMotorRPM(unsigned int rpm) {
  std::cout << "setMotorRPM to" << std::to_string(rpm) << std::endl;
}