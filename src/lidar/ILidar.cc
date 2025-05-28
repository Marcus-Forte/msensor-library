#include "ILidar.hh"

namespace msensor {

Scan3D::Scan3D() : points(pcl::make_shared<PointCloud3>()), timestamp(0) {}

Scan3DI::Scan3DI() : points(pcl::make_shared<PointCloud3I>()), timestamp(0) {}

} // namespace msensor