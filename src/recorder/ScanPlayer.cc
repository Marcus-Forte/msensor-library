#include "recorder/ScanPlayer.hh"

ScanPlayer::ScanPlayer(const std::filesystem::path &file) {
  if (!std::filesystem::exists(file)) {
    throw std::runtime_error("File does not exist: " + file.string());
  }
  record_file_.open(file);
}

bool ScanPlayer::next() {
  size_t msg_size;

  if (record_file_.peek() != EOF) {
    record_file_ >> msg_size;
    if (msg_size > read_buffer_.size()) {
      read_buffer_.resize(msg_size);
    }

    /// \todo Do we need to copy?
    record_file_.read(read_buffer_.data(), msg_size);
    entry_.ParseFromArray(read_buffer_.data(), msg_size);
    return true;
  }
  return false;
}

const sensors::RecordingEntry &ScanPlayer::getLastEntry() { return entry_; }