#include "recorder/ScanRecorder.hh"
#include "sensors.pb.h"
#include <ostream>
#include <string>
#include "timing/timing.hh"

namespace {
sensors::PointCloud3 toProtobuf(const Scan3D &scan) {
  sensors::PointCloud3 proto;
  for (const auto &pt : scan.points) {
    auto *proto_pt = proto.add_points();
    proto_pt->set_x(pt.x);
    proto_pt->set_y(pt.y);
    proto_pt->set_z(pt.z);
  }
  proto.set_timestamp(scan.timestamp);
  return proto;
}
} // namespace

ScanRecorder::ScanRecorder() : has_started_{false} {}

ScanRecorder::~ScanRecorder() { record_file_.close(); }

void ScanRecorder::start() {
  const auto cur_time = timing::getNowUs();

  record_file_.open("scan_" + std::to_string(cur_time) + ".pbscan",
                    std::ios::binary | std::ios::trunc);
  has_started_ = true;
}

void ScanRecorder::record(const Scan3D &scan) {
  if (!has_started_)
    return;
  auto proto_binary = toProtobuf(scan);
  
  proto_binary.set_timestamp(scan.timestamp);

  const auto bytes = proto_binary.ByteSizeLong();
  record_file_ << bytes;
  proto_binary.SerializeToOstream(&record_file_);
  record_file_ << std::flush;
}

void ScanRecorder::stop() {
  record_file_.close();
  has_started_ = false;
}