#include "lidar/simLidar.hh"
#include "timing/timing.hh"
#include <chrono>
#include <iostream>
#include <random>
#include <thread>
void SimLidar::init() { std::cout << "init" << std::endl; }

void SimLidar::startSampling() { std::cout << "startSampling" << std::endl; }
void SimLidar::stopSampling() { std::cout << "stopSampling" << std::endl; }

Scan3D SimLidar::getScan() {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> dis(-1.0, 1.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz.
  pcl::PointCloud<pcl::PointXYZI> pts;
  pts.points.emplace_back(dis(gen), dis(gen), 0);
  pts.points.emplace_back(dis(gen), dis(gen), 0);
  pts.points.emplace_back(dis(gen), dis(gen), 0);
  pts.points.emplace_back(dis(gen), dis(gen), 0);

  return {pts, timing::getNowUs()};
}