#include "sensors_server.hh"

#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

SensorsServer::SensorsServer() : scan_service_(100, 200) {}

SensorsServer::SensorsServer(size_t max_lidar_samples, size_t max_imu_samples)
    : scan_service_(max_lidar_samples, max_imu_samples) {}

void SensorsServer::start() {

  grpc::ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:50051",
                           ::grpc::InsecureServerCredentials());
  builder.RegisterService(&scan_service_);

  server_ = builder.BuildAndStart();
  std::cout << "Listening..." << std::endl;
}

void SensorsServer::stop() { server_->Shutdown(); }

void SensorsServer::publishScan(const std::shared_ptr<msensor::Scan3DI> &scan) {
  scan_service_.putScan(scan);
}
void SensorsServer::publishImu(
    const std::shared_ptr<msensor::IMUData> &imu_data) {
  scan_service_.putImuData(imu_data);
}