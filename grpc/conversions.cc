
#include "conversions.hh"

std::shared_ptr<msensor::Scan3DI> fromGRPC(const sensors::PointCloud3 &msg) {

  auto scan = std::make_shared<msensor::Scan3DI>();

  scan->points->reserve(msg.points_size());
  for (const auto &pt : msg.points()) {
    scan->points->emplace_back(pt.x(), pt.y(), pt.z());
  }
  scan->timestamp = msg.timestamp();

  return scan;
}

std::shared_ptr<msensor::IMUData> fromGRPC(const sensors::IMUData &msg) {
  auto imu_data = std::make_shared<msensor::IMUData>();
  imu_data->ax = msg.ax();
  imu_data->ay = msg.ay();
  imu_data->az = msg.az();
  imu_data->gx = msg.gx();
  imu_data->gy = msg.gy();
  imu_data->gz = msg.gz();
  imu_data->timestamp = msg.timestamp();
  return imu_data;
}

gl::PointCloud3 toGRPC(const std::shared_ptr<msensor::Scan3DI> &scan, float r,
                       float g, float b) {
  gl::PointCloud3 msg;

  msg.mutable_points()->Reserve(scan->points->size());
  for (const auto &pt : *scan->points) {
    auto *point = msg.add_points();
    point->set_x(pt.x);
    point->set_y(pt.y);
    point->set_z(pt.z);
    point->set_r(r);
    point->set_g(g);
    point->set_b(b);
  }

  return msg;
}
