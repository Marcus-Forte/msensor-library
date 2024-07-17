#pragma once

#include "ILidar.hh"
#include "sl_lidar_driver.h"
#include <string>

class RPLidar : public ILidar {
public:
  RPLidar(const std::string &serial_port);
  virtual ~RPLidar();

  void init() override;
  Point2Scan getScan() override;
  void setMotorRPM(unsigned int rpm) override;

private:
  sl::IChannel *channel_;
  sl::ILidarDriver *drv_;
};