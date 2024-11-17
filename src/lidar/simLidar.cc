#include "lidar/simLidar.hh"
#include <chrono>
#include <iostream>
#include <thread>

void SimLidar::init() { std::cout << "init" << std::endl; }

void SimLidar::startSampling() { std::cout << "startSampling" << std::endl; }
void SimLidar::stopSampling() { std::cout << "stopSampling" << std::endl; }

Scan3D SimLidar::getScan() {
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz.
  pcl::PointCloud<pcl::PointXYZI> pts;
  pts.points.emplace_back(1, 1, 0);
  pts.points.emplace_back(1, -1, 0);
  pts.points.emplace_back(-1, -1, 0);
  pts.points.emplace_back(1, -1, 0);

  return {pts, 0};
}