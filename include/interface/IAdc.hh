#pragma once

#include <cstdint>
#include <optional>

namespace msensor {

enum class AdcChannel : uint8_t {
  CHANNEL_0,
  CHANNEL_1,
  CHANNEL_2,
  CHANNEL_3,
};

struct AdcSample {
  double voltage;
  uint64_t timestamp;
};

class IAdc {
public:
  virtual ~IAdc() = default;

  /// Read a single-ended channel. Returns a sample with the voltage and the
  /// acquisition timestamp.
  virtual std::optional<AdcSample> readSingleEnded(AdcChannel channel) = 0;
};

} // namespace msensor
