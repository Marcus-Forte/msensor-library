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

void GLPublisher::attemptConnection(bool reset) {
  std::cout << "Slam Publisher attemting connection" << std::endl;
  stream_context_ = std::make_unique<grpc::ClientContext>();
  google::protobuf::Empty empty_response;
  writer_ = stub_->streamPointClouds(stream_context_.get(), &empty_response);

  grpc::ClientContext context;
  if (reset) {
    stub_->resetScene(&context, {}, &empty_response);
  }
}

void GLPublisher::resetScene() {
  grpc::ClientContext context;
  google::protobuf::Empty empty_response, empty_request;
  stub_->resetScene(&context, empty_request, &empty_response);
}

void GLPublisher::publishLines(const std::shared_ptr<msensor::Scan3DI> &src,
                               const std::shared_ptr<msensor::Scan3DI> &tgt,
                               float r, float g, float b) {
  size_t idx = 0;
  const auto &src_points = *src->points;
  const auto &tgt_points = *tgt->points;

  assert(src->points->size() == tgt->points->size());

  gl::LinesRequest lines;
  google::protobuf::Empty empty_response;
  grpc::ClientContext context;
  lines.set_r(r);
  lines.set_g(g);
  lines.set_b(b);
  for (size_t i = 0; i < src->points->size(); ++i) {
    auto *line = lines.add_lines();
    line->set_x0(src_points[i].x);
    line->set_y0(src_points[i].y);
    line->set_z0(src_points[i].z);

    line->set_x1(tgt_points[i].x);
    line->set_y1(tgt_points[i].y);
    line->set_z1(tgt_points[i].z);

    line->set_entity_name("line_" + std::to_string(idx++));
  }
  stub_->addLines(&context, lines, &empty_response);
}

void GLPublisher::publishScan(const std::shared_ptr<msensor::Scan3DI> &scan,
                              float r, float g, float b,
                              const std::string &name) {
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