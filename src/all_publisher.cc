#include "adc/ADS1115.hh"
#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "lidar/RPLidar.hh"
#include "sensors_server.hh"
#include "timing/timing.hh"
#include <filesystem>
#include <iostream>
#include <thread>

constexpr int kDefaultI2cBus = 1;
constexpr uint8_t kDefaultADSAddress = 0x48;
constexpr const char *kDefaultLidarDevice = "/dev/ttyUSB0";
constexpr uint64_t kLoopPeriodUs = 1000;

static void print_usage() {
  std::cout
      << "Usage: all_publisher [-b <i2c bus>] [-l <usb lidar device path>]"
      << std::endl;
}

int main(int argc, char **argv) {

  int opt;
  int bus = kDefaultI2cBus;
  std::filesystem::path lidar_device(kDefaultLidarDevice);

  while ((opt = getopt(argc, argv, "b:l:h")) != -1) {
    switch (opt) {
    case 'b':
      bus = std::strtol(optarg, nullptr, 0);
      break;
    case 'l':
      lidar_device = std::filesystem::path(optarg);
      break;
    case 'h':
    default:
      print_usage();
      return 0;
    }
  }

  if (!std::filesystem::exists(lidar_device)) {
    std::cerr << "Lidar device path does not exist: " << lidar_device
              << std::endl;
    return -1;
  }

  msensor::ADS1115 ads1115(bus, kDefaultADSAddress);
  ads1115.init(msensor::ADS1115::Gain::PLUS_MINUS_6_144,
               msensor::ADS1115::DataRate::SPS_8,
               static_cast<msensor::ADS1115::Channel>(0));

  msensor::RPLidar rplidar(lidar_device);
  rplidar.setMotorRPM(360);
  rplidar.init();

  msensor::ICM20948 icm20948(bus, ICM20948_ADDR0);
  icm20948.init();
  icm20948.calibrate();

  SensorsServer server;
  server.start();

  while (true) {
    const auto now = timing::getNowUs();

    const auto scan = rplidar.getScan();
    const auto imudata = icm20948.getImuData();
    auto adc_data = ads1115.readSingleEnded();

    constexpr float ExternalGain =
        (10.0f + 5.1f) / 5.1f; // 10k and 5.1k resistors
    adc_data->voltage *= ExternalGain;

    if (scan) {
      server.publishScan(scan);
      std::cout << "New Scan @ " << scan->timestamp
                << " Points: " << scan->points->size() << std::endl;
    }
    if (imudata) {
      server.publishImu(imudata.value());
    }
    if (adc_data) {
      server.publishAdc(adc_data.value());
    }

    const uint64_t remaining_us = kLoopPeriodUs - (timing::getNowUs() - now);
    if (remaining_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(remaining_us));
    } else {
      std::cout << "Loop overrun by " << -remaining_us << " us" << std::endl;
    }
  }
}