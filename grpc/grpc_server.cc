#include "grpc_server.hh"

#include <future>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <memory>

struct Point2 {
  float x;
  float y;
};

gRPCServer::gRPCServer() = default;

void gRPCServer::start() {

  task_ = std::async(std::launch::async, [this] {
    grpc::ServerBuilder builder;
    builder.AddListeningPort("0.0.0.0:50051",
                             ::grpc::InsecureServerCredentials());
    builder.RegisterService(&scan_service_);

    server_ = builder.BuildAndStart();
    std::cout << "Listening..." << std::endl;
    server_->Wait();
  });
}

void gRPCServer::stop() {
  server_->Shutdown();
  task_.get();
}

void gRPCServer::put_scan(const std::vector<Point2> &scan) {
  scan_service_.putScan(scan);
}