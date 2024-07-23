#pragma once
#include <vector>

struct Point2 {
  float x;
  float y;
};

class ILidar {
public:
  using Point2Scan = std::vector<Point2>;
  virtual ~ILidar() = default;

  virtual void init() = 0;
  virtual Point2Scan getScan() = 0;
  virtual void setMotorRPM(unsigned int) = 0;
};