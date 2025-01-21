#pragma once

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

} // namespace msensor
