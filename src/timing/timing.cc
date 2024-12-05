#include "timing/timing.hh"

#include <chrono>

namespace timing {

uint64_t getNowUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
} // namespace timing