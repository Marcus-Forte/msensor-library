#pragma once
#include <stdint.h>
#include <vector>

struct Point2 {
  float x;
  float y;
};

struct Scan2D {
  uint64_t timestamp;
  std::vector<Point2> points;
};

struct IMUData {
  uint64_t timestamp;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};