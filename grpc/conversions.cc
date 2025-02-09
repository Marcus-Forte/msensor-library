
#include "conversions.hh"

msensor::Scan3DI fromGRPC(const sensors::PointCloud3 &msg) {
  msensor::Scan3DI pointcloud;

  pointcloud.points.reserve(msg.points_size());
  for (const auto &pt : msg.points()) {
    pointcloud.points.emplace_back(pt.x(), pt.y(), pt.z());
  }
  pointcloud.timestamp = msg.timestamp();

  return pointcloud;
}

msensor::IMUData fromGRPC(const sensors::IMUData &msg) {

  return {
      msg.ax(), msg.ay(), msg.az(),        msg.gx(),
      msg.gy(), msg.gz(), msg.timestamp(),
  };
}

gl::PointCloud3 toGRPC(const msensor::PointCloud3 &scan, float r, float g,
                       float b) {
  gl::PointCloud3 msg;

  msg.mutable_points()->Reserve(scan.points.size());
  for (const auto &pt : scan.points) {
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
