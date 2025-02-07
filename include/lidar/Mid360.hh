#pragma once

#include <optional>
#include <string>

#include "ILidar.hh"
#include "imu/IImu.hh"

#include <boost/lockfree/spsc_queue.hpp>

namespace msensor {

class Mid360 : public ILidar {
public:
  enum class ScanPattern { Repetitive, NonRepetitive, LowFrameRate };
  enum class Mode { Normal, PowerSave };

  Mid360(const std::string &&config, size_t accumulate_scan_count);
  void init() override;
  Scan3DI getScan() override;
  void startSampling() override;
  void stopSampling() override;
  void setMode(Mode mode);

  void setScanPattern(ScanPattern pattern) const;
  std::optional<IMUData> getImuSample();

private:
  const std::string config_;
  Scan3DI pointclud_data_;

  boost::lockfree::spsc_queue<Scan3DI> scan_queue_;
  boost::lockfree::spsc_queue<IMUData> imu_queue_;

  const size_t accumulate_scan_count_;

  size_t scan_count_;

  uint32_t connection_handle_;
};

} // namespace msensor