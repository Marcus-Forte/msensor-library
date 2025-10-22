#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <stdint.h>

namespace msensor {

using Point3 = pcl::PointXYZ;
using Point3I = pcl::PointXYZI;

using PointCloud3 = pcl::PointCloud<Point3>;
using PointCloud3I = pcl::PointCloud<Point3I>;

/// \note We use `points` as shared_ptr to better interface with PCL.

/**
 * @brief 3D Pointcloud scan
 *
 */
struct Scan3D {
  Scan3D() : points(pcl::make_shared<PointCloud3>()), timestamp(0) {}
  PointCloud3::Ptr points;
  uint64_t timestamp;
};

/**
 * @brief 3D Pointcloud with Intensity.
 *
 */
struct Scan3DI {
  Scan3DI() : points(pcl::make_shared<PointCloud3I>()), timestamp(0) {}
  PointCloud3I::Ptr points;
  uint64_t timestamp;
};

class ILidar {
public:
  virtual ~ILidar() = default;

  virtual void init() = 0;
  virtual void startSampling() = 0;
  virtual void stopSampling() = 0;

  /**
   * @brief Return lidar scan.
   *
   * @return Scan3DI
   * @note The associated timestamp is assumed to be the time
   * point[0] was measured. Unit: ns (1/1000000000 sec).
   */
  virtual std::shared_ptr<Scan3DI> getScan() = 0;
};
} // namespace msensor