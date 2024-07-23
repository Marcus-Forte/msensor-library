
#include "opengl_srv_points.grpc.pb.h"
#include "points.grpc.pb.h"
#include <chrono>
#include <google/protobuf/empty.pb.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/server_builder.h>
#include <memory>
#include <thread>
// Simple application that subscribes to lidar service and publishes to opengl
// service: https://github.com/Marcus-Forte/learning-opengl

void printUsage() {
  std::cout << "gl_proxy [lidar server ip:port] [opengl server ip:port]"
            << std::endl;
}
int main(int argc, char **argv) {

  if (argc < 3) {
    printUsage();
    exit(-1);
  }
  const std::string lidarSrvIp(argv[1]);
  const std::string openGlSrvIp(argv[2]);

  auto channel =
      grpc::CreateChannel(lidarSrvIp, grpc::InsecureChannelCredentials());
  auto lidar_stub = lidar::LidarService::NewStub(channel);

  auto channel_opengl =
      grpc::CreateChannel(openGlSrvIp, grpc::InsecureChannelCredentials());
  auto opengl_stub = gl::addToScene::NewStub(channel_opengl);
  google::protobuf::Empty empty_response;
  std::unique_ptr<grpc::ClientContext> lidar_context =
      std::make_unique<grpc::ClientContext>();
  auto reader = lidar_stub->getScan(lidar_context.get(), empty_response);

  std::unique_ptr<grpc::ClientContext> opengl_context =
      std::make_unique<grpc::ClientContext>();

  auto writer =
      opengl_stub->streamPointClouds(opengl_context.get(), &empty_response);
  lidar::PointCloud3 msg;
  while (true) {

    if (!reader->Read(&msg)) {
      auto state = channel_opengl->GetState(true);
      std::cerr << "Error reading from lidar server " << lidarSrvIp
                << " code: " << state << std::endl;
      if (state == GRPC_CHANNEL_READY) {
        std::cout << "Reconnecting..." << std::endl;
        channel =
            grpc::CreateChannel(lidarSrvIp, grpc::InsecureChannelCredentials());
        lidar_context = std::make_unique<grpc::ClientContext>();
        lidar_stub = lidar::LidarService::NewStub(channel);
        reader = lidar_stub->getScan(lidar_context.get(), empty_response);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    auto gl_cloud = reinterpret_cast<gl::PointCloud3 *>(&msg);
    gl_cloud->set_entity_name("rplidar");
    if (!writer->Write(*gl_cloud)) {
      auto state = channel_opengl->GetState(true);
      std::cerr << "Error writing to opengl server " << openGlSrvIp
                << " code: " << state << std::endl;
      if (state == GRPC_CHANNEL_READY) {
        // Reconnect
        std::cout << "Reconnecting..." << std::endl;
        channel_opengl = grpc::CreateChannel(
            openGlSrvIp, grpc::InsecureChannelCredentials());
        opengl_context = std::make_unique<grpc::ClientContext>();
        opengl_stub = gl::addToScene::NewStub(channel_opengl);
        writer = opengl_stub->streamPointClouds(opengl_context.get(),
                                                &empty_response);
      }
    }
  }
}