#pragma once

#include "interface/ILidar.hh"

namespace msensor {

class SimLidar : public ILidar {
public:
  void init() override;
  void startSampling() override;
  void stopSampling() override;
  std::shared_ptr<Scan3DI> getScan() override;
};

} // namespace msensor