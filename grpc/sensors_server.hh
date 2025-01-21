#pragma once

#include "sensors_service.hh"
#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

class gRPCServer {
public:
  gRPCServer();

  void start();
  void stop();

  void put_scan(const msensor::Scan3D &scan);
  void put_imu(const msensor::IMUData &data);

private:
  ScanService scan_service_;
  std::unique_ptr<grpc::Server> server_;
};