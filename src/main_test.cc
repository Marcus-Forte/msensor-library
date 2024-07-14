#include "sl_lidar_driver.h"
#include <iostream>

using namespace sl;

void print_usage() {
  std::cout << "Usage: rplidar_publisher [usb device] [grpc server address]"
            << std::endl;
}

int main(int argc, char **argv) {

  sl_result op_result;

  IChannel *_channel;
  ILidarDriver *drv = *createLidarDriver();

  if (!drv) {
    fprintf(stderr, "insufficent memory, exit\n");
    exit(-2);
  }

  if (argc < 3) {
    print_usage();
    exit(0);
  }

  const std::string usb_port(argv[1]);
  const std::string grpc_server(argv[2]);

  std::cout << "Using Port: " << usb_port << std::endl;

  _channel = (*createSerialPortChannel(usb_port, 115200));
  sl_lidar_response_device_info_t devinfo;
  bool connectSuccess = false;
  if (SL_IS_OK((drv)->connect(_channel))) {
    op_result = drv->getDeviceInfo(devinfo);

    if (SL_IS_OK(op_result)) {
      connectSuccess = true;
      printf("SLAMTEC LIDAR S/N: ");
      for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
      }
      printf("\n"
             "Firmware Ver: %d.%02d\n"
             "Hardware Rev: %d\n",
             devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF,
             (int)devinfo.hardware_version);
    } else {
      delete drv;
      drv = NULL;
      std::cout << "Unable to connect to LIDAR!" << std::endl;
      exit(0);
    }

    drv->setMotorSpeed(0);
    // start scan...
    drv->startScan(0, 1);

    while (1) {
      sl_lidar_response_measurement_node_hq_t nodes[8192];
      size_t count = sizeof(nodes) / sizeof(nodes[0]);

      op_result = drv->grabScanDataHq(nodes, count, 1000);
      // std::cout << "Actual count: " << count;
      if (SL_IS_OK(op_result)) {
        drv->ascendScanData(nodes, count); // AKA Reorder
        std::cout << "OK!" << std::endl;
      } else {
        std::cout << "NOK!" << std::endl;
      }
    }
  }

  return 0;
}