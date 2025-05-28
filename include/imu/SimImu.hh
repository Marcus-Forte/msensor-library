#pragma once

#include "IImu.hh"

namespace msensor {
class SimImu : public IImu {
public:
  std::shared_ptr<IMUData> getImuData() override;
};

} // namespace msensor