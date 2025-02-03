
#include "gl_server.grpc.pb.h"
#include "sensors.grpc.pb.h"
#include <google/protobuf/empty.pb.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/server_builder.h>
#include <memory>
#include <thread>

#include "colormap.hh"

// Simple application that subscribes to lidar service and publishes to opengl
// service: https://github.com/Marcus-Forte/learning-opengl

static gl::PointCloud3 fromLidarService(sensors::PointCloud3 &scan_data) {

  // Zero only works if .proto are the same.
  // return reinterpret_cast<gl::PointCloud3 *>(&scan_data);

  gl::PointCloud3 ret;
  ret.mutable_points()->Reserve(scan_data.points_size());

  for (size_t i = 0; i < scan_data.points_size(); ++i) {
    auto *point = ret.add_points();
    point->set_x(scan_data.points(i).x());
    point->set_y(scan_data.points(i).y());
    point->set_z(scan_data.points(i).z());
    float pt_r;
    float pt_g;
    float pt_b;
    Int2RGB(scan_data.points(i).intensity(), pt_r, pt_g, pt_b);
    point->set_r(pt_r);
    point->set_g(pt_g);
    point->set_b(pt_b);
  }
  return ret;
}
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

  // Channels
  auto channel =
      grpc::CreateChannel(lidarSrvIp, grpc::InsecureChannelCredentials());
  auto lidar_stub = sensors::SensorService::NewStub(channel);

  auto channel_opengl =
      grpc::CreateChannel(openGlSrvIp, grpc::InsecureChannelCredentials());
  auto opengl_stub = gl::addToScene::NewStub(channel_opengl);

  google::protobuf::Empty empty_response;

  // Contexts
  std::unique_ptr<grpc::ClientContext> lidar_context =
      std::make_unique<grpc::ClientContext>();
  auto reader = lidar_stub->getScan(lidar_context.get(), empty_response);

  std::unique_ptr<grpc::ClientContext> opengl_context =
      std::make_unique<grpc::ClientContext>();

  auto writer =
      opengl_stub->streamPointClouds(opengl_context.get(), &empty_response);
  sensors::PointCloud3 msg;
  while (true) {

    if (!reader->Read(&msg)) {
      auto state = channel_opengl->GetState(true);
      std::cerr << "Error reading from lidar server " << lidarSrvIp
                << " code: " << state << std::endl;
      if (state == GRPC_CHANNEL_READY) {
        // Reconnect
        std::cout << "Reconnecting..." << std::endl;
        lidar_context = std::make_unique<grpc::ClientContext>();
        reader = lidar_stub->getScan(lidar_context.get(), empty_response);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    auto gl_cloud = fromLidarService(msg);
    gl_cloud.set_entity_name("lidar_scan");
    if (!writer->Write(gl_cloud)) {
      auto state = channel_opengl->GetState(true);
      std::cerr << "Error writing to opengl server " << openGlSrvIp
                << " code: " << state << std::endl;
      if (state == GRPC_CHANNEL_READY) {
        // Reconnect
        std::cout << "Reconnecting..." << std::endl;
        opengl_context = std::make_unique<grpc::ClientContext>();
        writer = opengl_stub->streamPointClouds(opengl_context.get(),
                                                &empty_response);
      }
    }
  }
}