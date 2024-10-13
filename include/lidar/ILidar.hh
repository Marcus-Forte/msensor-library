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

class ILidar {
public:
  virtual ~ILidar() = default;

  virtual void init() = 0;
  virtual Scan2D getScan() = 0;
  virtual void setMotorRPM(unsigned int) = 0;
};