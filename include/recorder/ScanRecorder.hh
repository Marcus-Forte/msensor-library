#pragma once

#include "lidar/ILidar.hh"
#include <fstream>

class ScanRecorder {
public:
  ScanRecorder();
  ~ScanRecorder();
  void start();
  void record(const Scan3D &scan);
  void stop();

private:
  std::ofstream record_file_;
  bool has_started_;
};