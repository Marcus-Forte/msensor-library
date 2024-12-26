#pragma once

#include <filesystem>

#include <sensors.pb.h>

class ScanPlayer {
public:
  ScanPlayer(const std::filesystem::path &file);

  bool next();
  const sensors::RecordingEntry &getLastEntry();

private:
  char *memory_map_;
  size_t offset_;
  size_t num_bytes_;
  sensors::RecordingEntry entry_;
};