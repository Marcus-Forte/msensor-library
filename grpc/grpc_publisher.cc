#include "grpc_publisher.hh"

gRPCPublisher::gRPCPublisher(const std::string &url) {
  channel_ = grpc::CreateChannel(url, grpc::InsecureChannelCredentials());
  stub_ = addToScene::NewStub(channel_);
}
