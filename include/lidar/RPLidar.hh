#pragma once

#include "ILidar.hh"
#include "sl_lidar_driver.h"
#include <string>

namespace msensor {

class RPLidar : public ILidar {
public:
  RPLidar(const std::string &serial_port);
  virtual ~RPLidar();

  void init() override;
  inline void startSampling() override {}
  inline void stopSampling() override {}
  std::shared_ptr<Scan3DI> getScan() override;
  void setMotorRPM(unsigned int rpm);

private:
  sl::IChannel *channel_;
  sl::ILidarDriver *drv_;
};

} // namespace msensor