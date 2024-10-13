#include "sensors.pb.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {

  bool simulate_time = false;
  bool publish_scan = false;
  std::string file;
  int opt;
  while ((opt = getopt(argc, argv, "tpf:")) != -1) {
    switch (opt) {
    case 't':
      std::cout << "Sim time" << std::endl;
      simulate_time = true;
      break;

    case 'p':
      std::cout << "Publish (TODO)" << std::endl;
      publish_scan = true;
      break;
    case 'f':
      file = optarg;
      break;

    default:
      std::cerr << "Usage: scan_parser [-f file] [-t sim time] [-p publish]"
                << std::endl;
      exit(0);
    }
  }

  if (!std::filesystem::exists(file)) {
    std::cerr << "File does not exist: " << file << std::endl;
    exit(-1);
  }

  std::ifstream pbscan(file, std::ios::in | std::ios::binary);
  size_t bytes;
  sensors::PointCloud3 scan;

  uint64_t time;
  int count = 0;
  int err_count = 0;
  while (pbscan.peek() != EOF) {
    pbscan >> bytes;
    std::string data(bytes, 0);
    pbscan.read(data.data(), bytes);
    if (!scan.ParseFromString(data)) {
      err_count++;
    } else {

      if (count > 0) {
        const auto delta_us = scan.timestamp() - time;
        if (simulate_time) {
          std::this_thread::sleep_for(std::chrono::milliseconds(delta_us));
          std::cout << scan.timestamp() << " pts: " << scan.points_size()
                    << std::endl;
        }
      }
      time = scan.timestamp();

      count++;
    }
  }

  std::cout << "Valid scans: " << count << std::endl;
  std::cout << "Errors scans: " << err_count << std::endl;
}