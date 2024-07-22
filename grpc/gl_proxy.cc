
#include "opengl_srv_points.grpc.pb.h"
#include "points.grpc.pb.h"
#include <chrono>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/server_builder.h>
#include <thread>
// Simple application that subscribes to lidar service and publishes to opengl
// service

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
  auto lidar_stub = LidarService::NewStub(channel);

  auto channel_opengl =
      grpc::CreateChannel(openGlSrvIp, grpc::InsecureChannelCredentials());
  auto opengl_stub = opengl::addToScene::NewStub(channel_opengl);

  grpc::ClientContext lidar_context;
  auto reader = lidar_stub->getScan(&lidar_context, {});

  grpc::ClientContext opengl_context;
  auto writer = opengl_stub->steamPointClouds(&opengl_context, {});
  PointCloud3 msg;
  opengl::PointCloud3 gl_pt;
  auto pt = gl_pt.add_points();
  pt->set_x(1);
  pt->set_y(2);
  pt->set_z(0);
  pt->set_r(1.0);

  while (true) {
    reader->Read(&msg);
    std::cout << "read: " << msg.points_size() << std::endl;

    // auto gl_msg = reinterpret_cast<opengl::PointCloud3 *>(&msg);

    writer->Write(gl_pt);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}