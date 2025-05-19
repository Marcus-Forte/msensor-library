#include "sensors_service.hh"
#include "timing/timing.hh"
#include <pcl/io/ply_io.h>

// #include "colormap.hh" // intensity -> RGB

ScanService::ScanService(size_t max_lidar_samples, size_t max_imu_samples)
    : scan_queue_(max_lidar_samples), imu_queue_(max_imu_samples) {}
grpc::Status
ScanService::getScan(::grpc::ServerContext *context,
                     const ::google::protobuf::Empty * /*request*/,
                     ::grpc::ServerWriter<sensors::PointCloud3> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected)
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client stream supported");

  std::cout << "Start Lidar scan stream." << std::endl;
  s_client_connected = true;
  while (!context->IsCancelled()) {

    while (!scan_queue_.empty()) {
      auto &scan = scan_queue_.front();
      sensors::PointCloud3 point_cloud;
      point_cloud.set_timestamp(scan.timestamp);
      for (const auto &point : scan.points) {
        auto pt = point_cloud.add_points();
        pt->set_x(point.x);
        pt->set_y(point.y);
        pt->set_z(point.z);
        pt->set_intensity(point.intensity);
      }
      writer->Write(point_cloud);
      scan_queue_.pop();
    }
  }
  std::cout << "Ending Lidar scan stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}

::grpc::Status
ScanService::savePLYScan(::grpc::ServerContext *context,
                         const ::sensors::saveFileRequest *request,
                         ::google::protobuf::Empty *response) {

  /// \todo check if stream is active. Save in PBscan or PCL?

  auto scan = scan_queue_.front();
  std::string filename;
  if (request->has_filename()) {
    filename = std::format("{}.ply", request->filename());
  } else {
    filename = std::format("scan_{}.ply", timing::getNowUs());
  }

  if (pcl::io::savePLYFileBinary<pcl::PointXYZI>(filename, scan.points) == 0) {
    std::cout << "Saved scan to:" << filename << std::endl;
    return ::grpc::Status::OK;
  }

  return ::grpc::Status(grpc::StatusCode::INTERNAL,
                        "Error saving scan to PLY file, no points");
}

void ScanService::putScan(const msensor::Scan3DI &scan) {

  if (scan_queue_.write_available() == 0) {
    scan_queue_.pop();
  }

  const auto res = scan_queue_.push(scan);

  if (res == false) {
    std::cerr << "Scan queue is full. Dropping scan." << std::endl;
  }
}

void ScanService::putImuData(const msensor::IMUData &imu_data) {
  if (imu_queue_.write_available() == 0) {
    imu_queue_.pop();
  }
  const auto res = imu_queue_.push(imu_data);

  if (res == false) {
    std::cerr << "Imu queue is full. Dropping imu data." << std::endl;
  }
}

::grpc::Status
ScanService::getImu(::grpc::ServerContext *context,
                    const ::google::protobuf::Empty *request,
                    ::grpc::ServerWriter<sensors::IMUData> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected)
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client supported");
  std::cout << "Start IMU data stream." << std::endl;
  s_client_connected = true;
  while (!context->IsCancelled()) {

    while (!imu_queue_.empty()) {
      const auto imu_data = imu_queue_.front();

      sensors::IMUData grpc_data;
      grpc_data.set_ax(imu_data.ax);
      grpc_data.set_ay(imu_data.ay);
      grpc_data.set_az(imu_data.az);
      grpc_data.set_gx(imu_data.gx);
      grpc_data.set_gy(imu_data.gy);
      grpc_data.set_gz(imu_data.gz);
      grpc_data.set_timestamp(imu_data.timestamp);
      writer->Write(grpc_data);
      imu_queue_.pop();
    }
  }
  std::cout << "Ending IMU data stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}