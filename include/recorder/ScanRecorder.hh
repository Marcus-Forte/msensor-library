#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"
#include <fstream>

class ScanRecorder {
public:
  ScanRecorder();
  ~ScanRecorder();
  void start();
  void start(const std::string &filename);
  void record(const Scan3D &scan);
  void record(const IMUData &imu);
  void stop();

private:
  std::ofstream record_file_;
  bool has_started_;
};