#pragma once
#include "points.grpc.pb.h"
struct Point2;
class ScanService : public lidar::LidarService::Service {
public:
  ScanService();
  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::google::protobuf::Empty *request,
          ::grpc::ServerWriter<lidar::PointCloud3> *writer) override;

  void putScan(const std::vector<Point2> &scan);

private:
  std::deque<std::vector<Point2>> scan_queue_;
};