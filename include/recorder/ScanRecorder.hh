#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"
#include <file/IFile.hh>

namespace msensor {

class ScanRecorder {
public:
  ScanRecorder(const std::shared_ptr<IFile> &file);
  ~ScanRecorder();

  /**
   * @brief Start the recording. Creates a file with the current timestamp.
   *
   */
  void start();

  /**
   * @brief Start the recording. Creates a file with a given name.
   *
   * @param filename
   */
  void start(const std::string &filename);

  /**
   * @brief Records a laser scan into scanfile. Thread-safe.
   *
   */
  void record(const std::shared_ptr<Scan3DI> &scan);

  /**
   * @brief Records an IMU data into scanfile. Thread-safe.
   *
   */
  void record(const std::shared_ptr<IMUData> &imu);

  /**
   * @brief Stops the recording.
   *
   */
  void stop();

private:
  std::shared_ptr<IFile> record_file_;
  bool has_started_;
};

} // namespace msensor