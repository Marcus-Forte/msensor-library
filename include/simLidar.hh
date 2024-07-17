#pragma once
#include "ILidar.hh"

class SimLidar : public ILidar {
public:
  void init() override;
  Point2Scan getScan() override;
  void setMotorRPM(unsigned int) override;
};