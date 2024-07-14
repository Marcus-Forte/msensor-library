#include "RPLidar.hh"
#include "grpc_publisher.hh"
#include <iostream>

void print_usage() {
  std::cout << "Usage: rplidar_publisher [usb device] [grpc server address]"
            << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 3) {
    print_usage();
    exit(0);
  }

  RPLidar lidar(argv[1]);
  lidar.init();

  const std::string grpc_server = argv[2];
  const auto publish_address = grpc_server + ":50051";
  std::cout << "Publishing to: " + publish_address << std::endl;
  gRPCPublisher publisher(publish_address);

  while (true) {
    auto scan = lidar.getScan();
    std::cout << "Scans pts: " << scan.size() << std::endl;
    publisher.send(scan);
  }
}