#pragma once

#include "interface/IImu.hh"

namespace msensor {
class SimImu : public IImu {
public:
  std::optional<IMUData> getImuData() override;
};

} // namespace msensor