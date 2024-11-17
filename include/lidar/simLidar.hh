#pragma once
#include "ILidar.hh"

class SimLidar : public ILidar {
public:
  void init() override;
  void startSampling() override;
  void stopSampling() override;
  Scan3D getScan() override;
};