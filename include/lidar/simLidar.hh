#pragma once

#include "ILidar.hh"

namespace msensor {

class SimLidar : public ILidar {
public:
  void init() override;
  void startSampling() override;
  void stopSampling() override;
  Scan3D getScan() override;
};

} // namespace msensor