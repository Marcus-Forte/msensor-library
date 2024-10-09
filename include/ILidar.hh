#pragma once
#include "SensorData.hh"

class ILidar {
public:
  virtual ~ILidar() = default;

  virtual void init() = 0;
  virtual Scan2D getScan() = 0;
  virtual void setMotorRPM(unsigned int) = 0;
};