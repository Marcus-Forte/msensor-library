#pragma once

#include "gl_server.grpc.pb.h"
#include <lidar/ILidar.hh>

namespace msensor {

class SlamPublisher {
public:
  SlamPublisher(const std::string &server_address);
  ~SlamPublisher();

  void publishScan(const msensor::PointCloud3 &scan);
  void publishMap(const msensor::PointCloud3 &map);
  void publishPose(float x, float y, float z);

private:
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<grpc::ClientContext> context_;
  std::unique_ptr<gl::addToScene::Stub> stub_;
  std::unique_ptr<::grpc::ClientWriter<::gl::PointCloud3>> writer_;

  void attemptConnection();
};

} // namespace msensor