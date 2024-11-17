#pragma once
#include "imu/IImu.hh"
#include "lidar/ILidar.hh"
#include "sensors.grpc.pb.h"

class ScanService : public sensors::SensorService::Service {
public:
  ScanService();
  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::google::protobuf::Empty *request,
          ::grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  ::grpc::Status getImu(::grpc::ServerContext *context,
                        const ::google::protobuf::Empty *request,
                        ::grpc::ServerWriter<sensors::IMUData> *writer) override;

  void putScan(const Scan3D &scan);
  void putImuData(const IMUData &imu_data);

private:
  std::deque<Scan3D> scan_queue_;
  std::deque<IMUData> imu_queue_;
};