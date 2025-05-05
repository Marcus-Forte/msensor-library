#pragma once

#include "gl_server.grpc.pb.h"
#include <lidar/ILidar.hh>

namespace msensor {

/**
 * @brief GL publisher class for publishing point clouds and poses to a GL
 * server.
 *
 */
class GLPublisher {
public:
  GLPublisher(const std::string &server_address);
  ~GLPublisher();

  void publishScan(const msensor::PointCloud3 &scan, float r = 0.0,
                   float g = 1.0, float b = 0.0,
                   const std::string &name = "scan");
  void publishPose(float x, float y, float z, float r = 0.0, float g = 0.0,
                   float b = 1.0, const std::string &name = "pose");

private:
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<grpc::ClientContext> context_;
  std::unique_ptr<gl::addToScene::Stub> stub_;
  std::unique_ptr<::grpc::ClientWriter<::gl::PointCloud3>> writer_;

  void attemptConnection(bool reset);
};

} // namespace msensor