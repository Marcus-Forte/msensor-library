#include "imu/icm-20948.h"
#include "imu/icm-20948_defs.h"
#include "sensors_server.hh"
#include "timing/timing.hh"
#include <getopt.h>
#include <iostream>
#include <thread>

void print_usage() {
  std::cout << "Usage: imu_publisher [i2c device] " << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    print_usage();
    exit(0);
  }

  auto i2c_device = std::atoi(argv[1]);

  ICM20948 icm20948(i2c_device, ICM20948_ADDR0);
  icm20948.init();

  icm20948.calibrate();

  SensorsServer server(10, 10);
  server.start();

  while (true) {

    auto acc_data = icm20948.get_acc_data();
    auto gyr_data = icm20948.get_gyro_data();
    auto dbl_acc_data = icm20948.convert_raw_data(acc_data, FACTOR_ACC_2G);
    auto dbl_gyr_data =
        icm20948.convert_raw_data(gyr_data, FACTOR_GYRO_500DPS_RADS);
    auto timestamp = timing::getNowUs();

    auto data = std::make_shared<msensor::IMUData>();
    data->timestamp = timestamp;
    data->ax = static_cast<float>(dbl_acc_data.x);
    data->ay = static_cast<float>(dbl_acc_data.y);
    data->az = static_cast<float>(dbl_acc_data.z);
    data->gx = static_cast<float>(dbl_gyr_data.x);
    data->gy = static_cast<float>(dbl_gyr_data.y);
    data->gz = static_cast<float>(dbl_gyr_data.z);

    server.publishImu(data);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  server.stop();
}
