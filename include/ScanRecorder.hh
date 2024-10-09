#pragma once

#include "ILidar.hh"
#include "points.pb.h"
#include <fstream>

class ScanRecorder {
public:
  ScanRecorder();
  ~ScanRecorder();
  void start();
  void record(const Scan2D &scan);
  void stop();

private:
  std::ofstream record_file_;
  bool has_started_;
};