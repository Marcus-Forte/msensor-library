#pragma once

#include <memory>
#include <stdint.h>

namespace msensor {

struct IMUData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  uint64_t timestamp;
};

class IImu {
public:
  virtual std::shared_ptr<IMUData> getImuData() = 0;
};

} // namespace msensor
