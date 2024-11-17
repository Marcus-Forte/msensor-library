#pragma once

#include <deque>
#include <optional>
#include <string>

#include "ILidar.hh"
#include "imu/IImu.hh"

class Mid360 : public ILidar {
public:
  enum class ScanPattern { Repetitive, NonRepetitive, LowFrameRate };
  enum class Mode { Normal, PowerSave };

  Mid360(const std::string &&config, size_t accumulate_scan_count);
  void init() override;
  Scan3D getScan() override;
  void startSampling() override;
  void stopSampling() override;
  void setMode(Mode mode);

  void setScanPattern(ScanPattern pattern) const;
  std::optional<IMUData> getImuSample();

private:
  const std::string config_;
  Scan3D pointclud_data_;
  std::deque<Scan3D> queue_;
  std::deque<IMUData> queue_imu_;

  const size_t queue_limit_ = 50;
  const size_t accumulate_scan_count_;

  size_t scan_count_;

  uint32_t connection_handle_;
};