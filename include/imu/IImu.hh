#pragma once

#include <stdint.h>

struct IMUData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  uint64_t timestamp;
};
