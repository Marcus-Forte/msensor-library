#include "sensors_service.hh"
#include "timing/timing.hh"
#include <pcl/io/ply_io.h>

constexpr uint32_t DEFAULT_QUEUE_SIZE = 100;
constexpr uint32_t MAX_QUEUE_SIZE = 100000;

static bool isQueueSizeValid(uint32_t queue_size) {
  return (queue_size > 0 && queue_size <= MAX_QUEUE_SIZE);
}

ScanService::ScanService() = default;

grpc::Status
ScanService::getScan(::grpc::ServerContext *context,
                     const ::sensors::SensorStreamRequest *request,
                     ::grpc::ServerWriter<sensors::PointCloud3> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected)
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client stream supported");

  std::cout << "Start Lidar scan stream." << std::endl;

  uint32_t queue_size = DEFAULT_QUEUE_SIZE;

  if (!request->has_queue_size()) {
    queue_size = DEFAULT_QUEUE_SIZE;
  } else if (isQueueSizeValid(request->queue_size())) {
    queue_size = request->queue_size();
  } else {
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "Invalid queue size requested.");
  }

  s_client_connected = true;

  // allocate queue
  scan_queue_ = std::make_shared<QueuePtrT<msensor::Scan3DI>>(queue_size);

  while (!context->IsCancelled()) {

    while (!scan_queue_->empty()) {
      auto &scan = scan_queue_->front();
      sensors::PointCloud3 point_cloud;

      point_cloud.set_timestamp(scan->timestamp);
      for (const auto &point : *scan->points) {
        auto pt = point_cloud.add_points();
        pt->set_x(point.x);
        pt->set_y(point.y);
        pt->set_z(point.z);
        pt->set_intensity(point.intensity);
      }
      writer->Write(point_cloud);
      /// \todo this infrige SPSC rule
      scan_queue_->pop();
    }
  }
  std::cout << "Ending Lidar scan stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}

::grpc::Status
ScanService::getImu(::grpc::ServerContext *context,
                    const ::sensors::SensorStreamRequest *request,
                    ::grpc::ServerWriter<sensors::IMUData> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected)
    return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                        "Only one client supported");
  std::cout << "Start IMU data stream." << std::endl;

  uint32_t queue_size = DEFAULT_QUEUE_SIZE;

  if (!request->has_queue_size()) {
    queue_size = DEFAULT_QUEUE_SIZE;
  } else if (isQueueSizeValid(request->queue_size())) {
    queue_size = request->queue_size();
  } else {
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "Invalid queue size requested.");
  }
  s_client_connected = true;

  // allocate queue
  imu_queue_ = std::make_shared<QueueT<msensor::IMUData>>(queue_size);

  while (!context->IsCancelled()) {

    while (!imu_queue_->empty()) {
      const auto imu_data = imu_queue_->front();

      sensors::IMUData grpc_data;
      grpc_data.set_ax(imu_data.ax);
      grpc_data.set_ay(imu_data.ay);
      grpc_data.set_az(imu_data.az);
      grpc_data.set_gx(imu_data.gx);
      grpc_data.set_gy(imu_data.gy);
      grpc_data.set_gz(imu_data.gz);
      grpc_data.set_timestamp(imu_data.timestamp);
      writer->Write(grpc_data);
      imu_queue_->pop();
    }
  }
  std::cout << "Ending IMU data stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}

::grpc::Status ScanService::GetAdc(::grpc::ServerContext *context,
                                   const ::sensors::AdcDataRequest *request,
                                   ::sensors::AdcData *response) {

  response->set_sample(adc_data_.voltage);
  response->set_timestamp(adc_data_.timestamp);
  return ::grpc::Status::OK;
}

::grpc::Status
ScanService::savePLYScan(::grpc::ServerContext *context,
                         const ::sensors::saveFileRequest *request,
                         ::google::protobuf::Empty *response) {

  /// \todo check if stream is active. Save in PBscan or PCL?
  if (!scan_queue_) {
    ::grpc::Status(grpc::StatusCode::INTERNAL, "No active stream");
  }
  auto scan = scan_queue_->front();
  std::string filename;
  if (request->has_filename()) {
    filename = std::format("{}.ply", request->filename());
  } else {
    filename = std::format("scan_{}.ply", timing::getNowUs());
  }

  if (pcl::io::savePLYFileBinary<pcl::PointXYZI>(filename, *scan->points) ==
      0) {
    std::cout << "Saved scan to:" << filename << std::endl;
    return ::grpc::Status::OK;
  }

  return ::grpc::Status(grpc::StatusCode::INTERNAL,
                        "Error saving scan to PLY file, no points");
}

void ScanService::putScan(const std::shared_ptr<msensor::Scan3DI> &scan) {
  if (!scan_queue_) {
    return;
  }

  if (scan_queue_->write_available() == 0) {
    scan_queue_->pop();
  }

  const auto res = scan_queue_->push(scan);

  if (res == false) {
    std::cerr << "Scan queue is full. Dropping scan." << std::endl;
  }
}

void ScanService::putImuData(msensor::IMUData imu_data) {

  if (!imu_queue_) {
    return;
  }

  if (imu_queue_->write_available() == 0) {
    imu_queue_->pop();
  }
  const auto res = imu_queue_->push(imu_data);

  if (res == false) {
    std::cerr << "Imu queue is full. Dropping imu data." << std::endl;
  }
}

void ScanService::putAdcData(msensor::AdcSample adc_data) {
  adc_data_ = adc_data;
}
