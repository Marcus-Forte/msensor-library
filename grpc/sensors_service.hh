#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"
#include "sensors.grpc.pb.h"
#include <boost/lockfree/spsc_queue.hpp>

/**
 * @brief Call the methods of this service to publish data via gRPC.
 *
 */
class ScanService : public sensors::SensorService::Service {
public:
  /**
   * @brief Construct a new Scan Service object
   *
   */
  ScanService();

  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::sensors::SensorStreamRequest *request,
          ::grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  ::grpc::Status
  getImu(::grpc::ServerContext *context,
         const ::sensors::SensorStreamRequest *request,
         ::grpc::ServerWriter<sensors::IMUData> *writer) override;

  /**
   * @brief Saves the oldest scan in the queue to a PLY file.
   */
  ::grpc::Status savePLYScan(::grpc::ServerContext *context,
                             const ::sensors::saveFileRequest *request,
                             ::google::protobuf::Empty *response) override;

  /**
   * @brief Puts a scan data in the server queue.
   *
   * @param scan
   */
  void putScan(const std::shared_ptr<msensor::Scan3DI> &scan);
  /**
   * @brief Puts IMU data into the server queue.
   *
   * @param imu_data
   */
  void putImuData(const std::shared_ptr<msensor::IMUData> &imu_data);

private:
  template <typename T>
  using QueueT = boost::lockfree::spsc_queue<std::shared_ptr<T>>;

  std::shared_ptr<QueueT<msensor::Scan3DI>> scan_queue_;
  std::shared_ptr<QueueT<msensor::IMUData>> imu_queue_;
};