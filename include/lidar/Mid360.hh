#pragma once

#include <string>

#include "ILidar.hh"
#include "imu/IImu.hh"

#include <boost/lockfree/spsc_queue.hpp>

namespace msensor {

/**
 * @brief This class represents a Mid360 lidar, with getters methods to receive
 * LiDAR data. It has an embedded IMU sensor, therefore we inherit IImu as well.
 *
 * \note Manual:
 * https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html#point-cloud-imu-data-protocol
 *
 */
class Mid360 : public ILidar, public IImu {
public:
  enum class ScanPattern { Repetitive, NonRepetitive, LowFrameRate };
  enum class Mode { Normal, PowerSave };

  /**
   * @brief Construct a new Mid360 object.
   *
   * @param config Configuration file to be loaded. \note The IP address of the
   * LiDAR is one of the configuration elements. Make sure your machine lies
   * within a reacheable subnet of the LiDAR.
   * @param accumulate_scan_count number of samples to accumulate when returning
   * data from getScan(). Typically the number of points per `getScan` is 96 *
   * `accumulate_scan_count`.
   */
  Mid360(const std::string &&config, size_t accumulate_scan_count);
  void init() override;
  std::shared_ptr<Scan3DI> getScan() override;
  std::shared_ptr<IMUData> getImuData() override;
  void startSampling() override;
  void stopSampling() override;
  void setMode(Mode mode);

  void setScanPattern(ScanPattern pattern) const;

private:
  const std::string config_;
  std::shared_ptr<Scan3DI> accumulated_pointcloud_data_;

  boost::lockfree::spsc_queue<std::shared_ptr<Scan3DI>> scan_queue_;
  boost::lockfree::spsc_queue<std::shared_ptr<IMUData>> imu_queue_;

  const size_t accumulate_scan_count_;

  size_t scan_count_;

  uint32_t connection_handle_;
};

} // namespace msensor