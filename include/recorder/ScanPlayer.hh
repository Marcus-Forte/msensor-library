#pragma once

#include <filesystem>
#include <fstream>

#include <sensors.pb.h>

class ScanPlayer {
public:
  ScanPlayer(const std::filesystem::path &file);

  bool next();
  const sensors::RecordingEntry &getLastEntry();

private:
  std::ifstream record_file_;
  std::vector<char> read_buffer_;
  sensors::RecordingEntry entry_;
};