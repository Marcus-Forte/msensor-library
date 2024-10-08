#include "ScanRecorder.hh"
#include "ILidar.hh"
#include "points.pb.h"
#include <chrono>
#include <ostream>
#include <string>

namespace {
lidar::PointCloud3 toProtobuf(const ILidar::Point2Scan &scan) {
  lidar::PointCloud3 proto;
  for (const auto &pt : scan) {
    auto *proto_pt = proto.add_points();
    proto_pt->set_x(pt.x);
    proto_pt->set_y(pt.y);
    proto_pt->set_z(0);
  }
  return proto;
}
} // namespace

ScanRecorder::ScanRecorder() : has_started_{false} {}

ScanRecorder::~ScanRecorder() { record_file_.close(); }

void ScanRecorder::start() {
  const auto cur_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

  record_file_.open("scan_" + std::to_string(cur_time) + ".pbscan",
                    std::ios::binary | std::ios::trunc);
  has_started_ = true;
}

void ScanRecorder::record(const ILidar::Point2Scan &scan) {
  if (!has_started_)
    return;
  auto proto_binary = toProtobuf(scan);
  proto_binary.set_timestamp(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());

  const size_t bytes = proto_binary.ByteSizeLong();
  record_file_ << bytes;
  proto_binary.SerializeToOstream(&record_file_);
  record_file_ << std::flush;
}

void ScanRecorder::stop() {
  record_file_.close();
  has_started_ = false;
}