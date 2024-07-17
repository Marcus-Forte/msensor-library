#include "ILidar.hh"
#include "RPLidar.hh"
#include "grpc_publisher.hh"
#include "simLidar.hh"
#include <filesystem>
#include <iostream>

void print_usage() {
  std::cout << "Usage: rplidar_publisher [serial device path || \"sim\" ] "
               "[grpc server address]"
            << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 3) {
    print_usage();
    exit(0);
  }
  std::unique_ptr<ILidar> lidar;
  if (std::string(argv[1]) == "sim") {
    lidar = std::make_unique<SimLidar>();
  } else {
    if (!std::filesystem::exists(argv[1])) {
      std::cerr << "Device: " << argv[1] << " does not exist. Exiting..."
                << std::endl;
      exit(-1);
    }
    lidar = std::make_unique<RPLidar>(argv[1]);
  }

  lidar->init();
  lidar->setMotorRPM(360);
  const std::string grpc_server = argv[2];
  const auto publish_address = grpc_server + ":50051";
  std::cout << "Publishing to: " + publish_address << std::endl;
  gRPCPublisher publisher(publish_address);

  while (true) {
    const auto scan = lidar->getScan();
    std::cout << "Scans pts: " << scan.size() << std::endl;
    publisher.send(scan);
  }
}