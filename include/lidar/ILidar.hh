#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <stdint.h>

namespace msensor {

using Point3 = pcl::PointXYZ;
using Point3I = pcl::PointXYZI;

using PointCloud3 = pcl::PointCloud<Point3>;
using PointCloud3I = pcl::PointCloud<Point3I>;

/**
 * @brief 3D Scan.
 *
 */
struct Scan3D {
  PointCloud3 points;
  uint64_t timestamp;
};

/**
 * @brief 3D Scan with Intensity.
 *
 */
struct Scan3DI {
  PointCloud3I points;
  uint64_t timestamp;
};

class ILidar {
public:
  virtual ~ILidar() = default;

  virtual void init() = 0;
  virtual void startSampling() = 0;
  virtual void stopSampling() = 0;

  /** Return lidar scan. Associated timestamp is assumed to be the time
   * point[0] was measured. Unit: ns (1/1000000000 sec) */
  virtual Scan3DI getScan() = 0;
};
} // namespace msensor