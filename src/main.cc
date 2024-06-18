#include <iostream>
#include "sl_lidar_driver.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "converter.hh"

#include "grpc_publisher.hh"

using namespace sl;
int main(int argc, char **argv)
{

  const char *opt_is_channel = NULL;
  const char *opt_channel = NULL;
  const char *opt_channel_param_first = NULL;
  sl_u32 opt_channel_param_second = 0;
  sl_u32 baudrateArray[2] = {115200, 256000};
  sl_result op_result;
  int opt_channel_type = CHANNEL_TYPE_SERIALPORT;

  bool useArgcBaudrate = false;

  IChannel *_channel;
  ILidarDriver *drv = *createLidarDriver();

  if (!drv)
  {
    fprintf(stderr, "insufficent memory, exit\n");
    exit(-2);
  }

  std::string port = "/dev/tty.usbserial-0001";
  if(argc > 1)
    port = argv[1];

  std::cout << "Using Port: " << port << std::endl;

  _channel = (*createSerialPortChannel(port, 115200));
  sl_lidar_response_device_info_t devinfo;
  bool connectSuccess = false;
  if (SL_IS_OK((drv)->connect(_channel)))
  {
    op_result = drv->getDeviceInfo(devinfo);

    if (SL_IS_OK(op_result))
    {
      connectSuccess = true;
      printf("SLAMTEC LIDAR S/N: ");
      for (int pos = 0; pos < 16; ++pos)
      {
        printf("%02X", devinfo.serialnum[pos]);
      }
      printf("\n"
             "Firmware Ver: %d.%02d\n"
             "Hardware Rev: %d\n",
             devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF, (int)devinfo.hardware_version);
    }
    else
    {
      delete drv;
      drv = NULL;
      std::cout << "Unable to connect to LIDAR!" << std::endl;
      exit(0);
    }
    
    drv->setMotorSpeed(0);
    // start scan...
    drv->startScan(0, 1);

    gRPCPublisher publisher("localhost:50051");

    while (1)
    {
      sl_lidar_response_measurement_node_hq_t nodes[8192];
      size_t count = sizeof(nodes) / sizeof(nodes[0]);

      op_result = drv->grabScanDataHq(nodes, count, 1000);
      // std::cout << "Actual count: " << count;
      if (SL_IS_OK(op_result))
      {
        std::cout << "OK!\n";
        // TODO convert to pointcloud!
        // AKA Reorder
        drv->ascendScanData(nodes, count);
        // for (int pos = 0; pos < (int)count; ++pos)
        // {
        //     printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
        //            (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : " N ",
        //            (nodes[pos].angle_z_q14 * M_PI_4)  / 16384.f,
        //            nodes[pos].dist_mm_q2 / 4.0f,
        //            nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        // }

        auto pointcloud = toPointCloud(nodes, count);
        // toFile(pointcloud);
        publisher.send<Point2>(pointcloud);
      }
      else
      {
        std::cout << "NOK!\n";
      }
    }
  }

  return 0;
}