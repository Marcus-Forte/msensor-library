#pragma once
#include "ILidar.hh"

class SimLidar : public ILidar {
public:
  void init() override;
  Scan2D getScan() override;
  void setMotorRPM(unsigned int) override;
};