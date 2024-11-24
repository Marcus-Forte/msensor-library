#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "lidar/RPLidar.hh"
#include "lidar/simLidar.hh"
#include "recorder/ScanRecorder.hh"
#include "sensors_server.hh"
#include <chrono>
#include <filesystem>
#include <future>
#include <getopt.h>
#include <iostream>
#include <mutex>
#include <thread>

std::mutex g_mutex;

// 100 Hz
void ImuLoop(gRPCServer &server) {
  std::cout << "Start imu loop" << std::endl;
  ICM20948 icm20948(1, ICM20948_ADDR0);
  icm20948.init();

  icm20948.calibrate();
  while (true) {

    auto acc_data = icm20948.get_acc_data();
    auto gyr_data = icm20948.get_gyro_data();
    auto dbl_acc_data = icm20948.convert_raw_data(acc_data, FACTOR_ACC_2G);
    auto dbl_gyr_data =
        icm20948.convert_raw_data(gyr_data, FACTOR_GYRO_500DPS_RADS);
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    IMUData data;
    data.timestamp = timestamp;
    data.ax = static_cast<float>(dbl_acc_data.x);
    data.ay = static_cast<float>(dbl_acc_data.y);
    data.az = static_cast<float>(dbl_acc_data.z);
    data.gx = static_cast<float>(dbl_gyr_data.x);
    data.gy = static_cast<float>(dbl_gyr_data.y);
    data.gz = static_cast<float>(dbl_gyr_data.z);
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      server.put_imu(data);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
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
    dynamic_cast<RPLidar *>(lidar.get())->setMotorRPM(360);
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

  gRPCServer server;
  server.start();

  auto imu_loop =
      std::async(std::launch::async, [&server]() { ImuLoop(server); });

  while (true) {
    const auto scan = lidar->getScan();
    std::cout << "Scans pts: " << scan.points.size() << std::endl;
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      server.put_scan(scan);
    }
    recorder.record(scan);
  }

  server.stop();
}