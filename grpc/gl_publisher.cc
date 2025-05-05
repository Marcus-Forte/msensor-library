#include "gl_publisher.hh"
#include "conversions.hh"
#include <grpcpp/grpcpp.h>

namespace msensor {

GLPublisher::GLPublisher(const std::string &server_address) {
  channel_ =
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());

  stub_ = gl::addToScene::NewStub(channel_);

  attemptConnection(true);
}

GLPublisher::~GLPublisher() = default;

void GLPublisher::attemptConnection(bool reset) {
  std::cout << "Slam Publisher attemting connection" << std::endl;
  context_ = std::make_unique<grpc::ClientContext>();
  google::protobuf::Empty empty_response;
  writer_ = stub_->streamPointClouds(context_.get(), &empty_response);

  auto context = std::make_unique<grpc::ClientContext>();
  if (reset) {
    stub_->resetScene(context.get(), {}, &empty_response);
  }
}

void GLPublisher::publishScan(const msensor::PointCloud3 &scan, float r,
                              float g, float b, const std::string &name) {
  // green
  auto gl_cloud = toGRPC(scan, r, g, b);
  gl_cloud.set_entity_name(name);
  if (!writer_->Write(gl_cloud)) {
    attemptConnection(false);
  }
}

void GLPublisher::publishPose(float x, float y, float z, float r, float g,
                              float b, const std::string &name) {
  // blue
  gl::PointCloud3 pose;
  pose.set_entity_name(name);
  auto point = pose.add_points();
  point->set_x(x);
  point->set_y(y);
  point->set_z(z);
  point->set_r(r);
  point->set_g(g);
  point->set_b(b);
  pose.set_point_size(25.0);

  if (!writer_->Write(pose)) {
    attemptConnection(false);
  }
}

} // namespace msensor