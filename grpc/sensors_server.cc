#include "sensors_server.hh"

#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

gRPCServer::gRPCServer() = default;

void gRPCServer::start() {

  grpc::ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:50051",
                           ::grpc::InsecureServerCredentials());
  builder.RegisterService(&scan_service_);

  server_ = builder.BuildAndStart();
  std::cout << "Listening..." << std::endl;
}

void gRPCServer::stop() { server_->Shutdown(); }

void gRPCServer::put_scan(const msensor::Scan3D &scan) {
  scan_service_.putScan(scan);
}
void gRPCServer::put_imu(const msensor::IMUData &imu_data) {
  scan_service_.putImuData(imu_data);
}