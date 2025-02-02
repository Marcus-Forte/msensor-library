#pragma once

#include "IImu.hh"

namespace msensor {
class SimImu : public IImu {
public:
  IMUData getImuData() override;
};

} // namespace msensor