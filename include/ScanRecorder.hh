#pragma once

#include "ILidar.hh"
#include "points.pb.h"
#include <fstream>

class ScanRecorder {
public:
  ScanRecorder();
  ~ScanRecorder();
  void start();
  void record(const ILidar::Point2Scan &scan);
  void stop();

private:
  std::ofstream record_file_;
  lidar::PointCloud3Series series_;
  bool has_started_;
};