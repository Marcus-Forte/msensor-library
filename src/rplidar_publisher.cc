#include "file/File.hh"
#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "lidar/RPLidar.hh"
#include "recorder/ScanRecorder.hh"
#include "sensors_server.hh"
#include "timing/timing.hh"
#include <filesystem>
#include <future>
#include <getopt.h>
#include <iostream>
#include <thread>

// 100 Hz
void ImuLoop(SensorsServer &server, msensor::ScanRecorder &recorder) {
  constexpr uint64_t sample_period_us = 10000; // 100 hz
  std::cout << "Start imu loop" << std::endl;
  msensor::ICM20948 icm20948(1, ICM20948_ADDR0);
  icm20948.init();

  icm20948.calibrate();
  while (true) {

     const auto now = timing::getNowUs();

    auto data = icm20948.getImuData();

    recorder.record(data);
    server.publishImu(data);

    const auto remaining_us = sample_period_us - (timing::getNowUs() - now);
    if (remaining_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(remaining_us));
    } else {
      std::cout << "IMU loop overrun by " << -remaining_us << " us"
                << std::endl;
    }
  }
}
void print_usage() {
  std::cout << "Usage: rplidar_publisher [serial device path]  [-r record] "
            << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    print_usage();
    exit(0);
  }

  std::unique_ptr<msensor::ILidar> lidar;
  if (!std::filesystem::exists(argv[1])) {
    std::cerr << "Device: " << argv[1] << " does not exist. Exiting..."
              << std::endl;
    exit(-1);
  }
  lidar = std::make_unique<msensor::RPLidar>(argv[1]);
  dynamic_cast<msensor::RPLidar *>(lidar.get())->setMotorRPM(360);

  auto file = std::make_shared<File>();
  msensor::ScanRecorder recorder(file);

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

  SensorsServer server(10, 10);
  server.start();

  auto imu_loop = std::async(std::launch::async, [&server, &recorder]() {
    ImuLoop(server, recorder);
  });

  while (true) {
    const auto scan = lidar->getScan();
    std::cout << "New Scan @ " << scan->timestamp
              << " Points: " << scan->points->size() << std::endl;

    recorder.record(scan);
    server.publishScan(scan);
  }

  server.stop();
}