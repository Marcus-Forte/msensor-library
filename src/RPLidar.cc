#include "RPLidar.hh"
#include "ILidar.hh"
#include "sl_lidar_driver.h"
#include <cmath>
#include <format>
#include <iostream>
#include <stdexcept>

constexpr uint32_t g_baudRate = 115200;

ILidar::Point2Scan
toPointVector(const sl_lidar_response_measurement_node_hq_t *nodes, int count) {
  std::vector<Point2> points;
  points.reserve(count);
  int idx = 0;
  for (int pos = 0; pos < (int)count; ++pos) {
    if (nodes[pos].quality < 40)
      continue;

    float angle_in_pi = (nodes[pos].angle_z_q14 * M_PI_2) / 16384.f;
    const float dist_m = nodes[pos].dist_mm_q2 / 4000.0f;
    float x = -cos(angle_in_pi) * dist_m;
    float y = sin(angle_in_pi) * dist_m;
    Point2 pt;
    pt.x = x;
    pt.y = y;
    points.push_back(pt);
  }
  return points;
}

RPLidar::RPLidar(const std::string &serial_port) {

  drv_ = *sl::createLidarDriver();

  if (!drv_) {
    fprintf(stderr, "insufficent memory, exit\n");
    exit(-2);
  }

  channel_ = *sl::createSerialPortChannel(serial_port, g_baudRate);
}
RPLidar::~RPLidar() = default;

void RPLidar::init() {

  if (!SL_IS_OK((drv_)->connect(channel_))) {
    throw std::runtime_error("Unable to connect to LiDAR");
  }
  sl_lidar_response_device_info_t devinfo;
  auto op_result = drv_->getDeviceInfo(devinfo);
  if (SL_IS_OK(op_result)) {
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
      printf("%02X", devinfo.serialnum[pos]);
    }
    std::cout << std::format("Firmware Ver: {}.{}"
                             "Hardware Rev: {}",
                             devinfo.firmware_version >> 8,
                             devinfo.firmware_version & 0xFF,
                             (int)devinfo.hardware_version)
              << std::endl;
  }

  sl::LidarMotorInfo motorinfo;
  drv_->getMotorInfo(motorinfo);
  std::cout << std::format("Motor info: desired speed: {}, min_speed: {}, "
                           "max_speed: {}, ctrl_support: {}",
                           motorinfo.desired_speed, motorinfo.min_speed,
                           motorinfo.max_speed,
                           static_cast<int>(motorinfo.motorCtrlSupport));

  drv_->setMotorSpeed(0);
  drv_->startScan(0, 1);
}

ILidar::Point2Scan RPLidar::getScan() {

  sl_lidar_response_measurement_node_hq_t nodes[8192];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);

  auto result = drv_->grabScanDataHq(nodes, count, 1000);
  if (SL_IS_OK(result)) {
    drv_->ascendScanData(nodes, count); // AKA Reorder
    return toPointVector(nodes, count);
  } else {
    return {};
  }
}

void RPLidar::setMotorRPM(unsigned int rpm) { drv_->setMotorSpeed(rpm); }