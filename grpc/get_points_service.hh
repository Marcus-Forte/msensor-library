#pragma once
#include "SensorData.hh"
#include "points.grpc.pb.h"

class ScanService : public lidar::LidarService::Service {
public:
  ScanService();
  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::google::protobuf::Empty *request,
          ::grpc::ServerWriter<lidar::PointCloud3> *writer) override;

  ::grpc::Status getImu(::grpc::ServerContext *context,
                        const ::google::protobuf::Empty *request,
                        ::grpc::ServerWriter<lidar::IMUData> *writer) override;

  void putScan(const Scan2D &scan);
  void putImuData(const IMUData &imu_data);

private:
  std::deque<Scan2D> scan_queue_;
  std::deque<IMUData> imu_queue_;
};