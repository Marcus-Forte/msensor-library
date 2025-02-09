#include "slam_publisher.hh"
#include "conversions.hh"
#include <grpcpp/grpcpp.h>

namespace msensor {

SlamPublisher::SlamPublisher(const std::string &server_address) {
  channel_ =
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());

  stub_ = gl::addToScene::NewStub(channel_);

  attemptConnection();
}

SlamPublisher::~SlamPublisher() = default;

void SlamPublisher::attemptConnection() {
  std::cout << "Slam Publisher attemting connection" << std::endl;
  context_ = std::make_unique<grpc::ClientContext>();
  google::protobuf::Empty empty_response;
  writer_ = stub_->streamPointClouds(context_.get(), &empty_response);

  auto context = std::make_unique<grpc::ClientContext>();
  stub_->resetScene(context.get(), {}, &empty_response);
}

void SlamPublisher::publishScan(const msensor::PointCloud3 &scan) {
  // green
  auto gl_cloud = toGRPC(scan, 0.0, 1.0, 0.0);
  gl_cloud.set_entity_name("slam_scan");
  if (!writer_->Write(gl_cloud)) {
    attemptConnection();
  }
}

/// \todo publish map increments only (unnamed gl::points)
void SlamPublisher::publishMap(const msensor::PointCloud3 &map) {
  // red
  auto gl_cloud = toGRPC(map, 1.0, 0.0, 0.0);
  gl_cloud.set_entity_name("slam_map");
  if (!writer_->Write(gl_cloud)) {
    attemptConnection();
  }
}

void SlamPublisher::publishPose(float x, float y, float z) {
  // blue
  gl::PointCloud3 pose;
  pose.set_entity_name("slam_pose");
  auto point = pose.add_points();
  point->set_x(x);
  point->set_y(y);
  point->set_z(z);
  point->set_r(0.0);
  point->set_g(0.0);
  point->set_b(1.0);
  pose.set_point_size(25.0);

  if (!writer_->Write(pose)) {
    attemptConnection();
  }
}

} // namespace msensor