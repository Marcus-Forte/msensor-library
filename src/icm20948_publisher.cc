#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "sensors_server.hh"
#include "timing/timing.hh"
#include <getopt.h>
#include <iostream>
#include <thread>

constexpr uint64_t sample_period_us = 10000; // 100 hz

int SetRealTimePriority() {
  struct sched_param sch;
  sch.sched_priority = 99;
  return sched_setscheduler(0, SCHED_FIFO, &sch);
}

void print_usage() {
  std::cout << "Usage: imu_publisher [i2c device] " << std::endl;
}

int main(int argc, char **argv) {

  SetRealTimePriority();

  if (argc < 2) {
    print_usage();
    exit(0);
  }

  auto i2c_device = std::atoi(argv[1]);

  msensor::ICM20948 icm20948(i2c_device, ICM20948_ADDR0);
  icm20948.init();

  icm20948.calibrate();

  SensorsServer server(10, 10);
  server.start();

  while (true) {
    const auto now = timing::getNowUs();

    auto data = icm20948.getImuData();

    server.publishImu(data);

    const auto remaining_us = sample_period_us - (timing::getNowUs() - now);
    if (remaining_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(remaining_us));
    } else {
      std::cout << "IMU loop overrun by " << -remaining_us << " us"
                << std::endl;
    }
  }

  server.stop();
}
