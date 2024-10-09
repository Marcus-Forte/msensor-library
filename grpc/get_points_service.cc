#include "get_points_service.hh"
#include <chrono>
#include <grpcpp/support/status.h>
#include <thread>

constexpr size_t g_maxSamples = 10;
constexpr size_t g_maxImuSamples = 400;

ScanService::ScanService() = default;

grpc::Status
ScanService::getScan(::grpc::ServerContext *context,
                     const ::google::protobuf::Empty * /*request*/,
                     ::grpc::ServerWriter<lidar::PointCloud3> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected)
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client supported");

  std::cout << "Start Lidar scan stream." << std::endl;
  s_client_connected = true;
  while (!context->IsCancelled()) {

    while (!scan_queue_.empty()) {
      auto scan = scan_queue_.front();
      lidar::PointCloud3 point_cloud;
      point_cloud.set_timestamp(scan.timestamp);
      for (const auto &point : scan.points) {
        auto pt = point_cloud.add_points();
        pt->set_x(point.x);
        pt->set_y(point.y);
        // 2D Lidar
      }
      writer->Write(point_cloud);
      scan_queue_.pop_front();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  s_client_connected = false;
  return ::grpc::Status::OK;
}

void ScanService::putScan(const Scan2D &scan) {
  scan_queue_.push_front(scan);
  if (scan_queue_.size() > g_maxSamples) {
    scan_queue_.pop_back();
  }
}

void ScanService::putImuData(const IMUData &imu_data) {
  imu_queue_.push_front(imu_data);
  if (imu_queue_.size() > g_maxImuSamples) {
    imu_queue_.pop_back();
  }
}

::grpc::Status
ScanService::getImu(::grpc::ServerContext *context,
                    const ::google::protobuf::Empty *request,
                    ::grpc::ServerWriter<lidar::IMUData> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected)
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client supported");
  std::cout << "Start IMU data stream." << std::endl;
  s_client_connected = true;
  while (!context->IsCancelled()) {

    while (!imu_queue_.empty()) {
      const auto imu_data = imu_queue_.front();

      lidar::IMUData grpc_data;
      grpc_data.set_ax(imu_data.ax);
      grpc_data.set_ay(imu_data.ay);
      grpc_data.set_az(imu_data.az);
      grpc_data.set_gx(imu_data.gx);
      grpc_data.set_gy(imu_data.gy);
      grpc_data.set_gz(imu_data.gz);
      grpc_data.set_timestamp(imu_data.timestamp);
      writer->Write(grpc_data);
      imu_queue_.pop_front();
    }
  }
  s_client_connected = false;
  return ::grpc::Status::OK;
}