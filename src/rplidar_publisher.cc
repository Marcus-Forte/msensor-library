#include "ILidar.hh"
#include "RPLidar.hh"
#include "ScanRecorder.hh"
#include "grpc_server.hh"
#include "simLidar.hh"
#include <filesystem>
#include <getopt.h>
#include <iostream>

void print_usage() {
  std::cout
      << "Usage: rplidar_publisher [serial device path || \"sim\" [-r] record] "
      << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 2) {
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

  ScanRecorder recorder;

  bool record_scans = false;
  int opt;
  while ((opt = getopt(argc, argv, "r")) != -1) {
    switch (opt) {
    case 'r':
      record_scans = true;
      std::cout << "Recording scan enabled" << std::endl;
      recorder.start();
      break;
    }
  }

  lidar->init();
  lidar->setMotorRPM(360);
  gRPCServer server;
  server.start();

  while (true) {
    const auto scan = lidar->getScan();
    std::cout << "Scans pts: " << scan.size() << std::endl;
    server.put_scan(scan);
    recorder.record(scan);
  }

  server.stop();
}