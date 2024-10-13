#pragma once

#include <stdint.h>

struct IMUData {
  uint64_t timestamp;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};
