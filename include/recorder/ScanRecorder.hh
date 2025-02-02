#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"
#include <file/IFile.hh>

namespace msensor {

class ScanRecorder {
public:
  ScanRecorder(const std::shared_ptr<IFile> &file);
  ~ScanRecorder();
  void start();
  void start(const std::string &filename);
  void record(const Scan3DI &scan);
  void record(const IMUData &imu);
  void stop();

private:
  std::shared_ptr<IFile> record_file_;
  bool has_started_;
};

} // namespace msensor