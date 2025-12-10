#pragma once

#include <cstdint>
#include <optional>

namespace msensor {

/**
 * @brief Supported ADC single-ended channels.
 */
enum class AdcChannel : uint8_t {
  CHANNEL_0,
  CHANNEL_1,
  CHANNEL_2,
  CHANNEL_3,
};

/**
 * @brief ADC measurement sample.
 */
struct AdcSample {
  float voltage;    ///< Measured voltage in volts.
  uint64_t timestamp; ///< Acquisition time in nanoseconds.
};

/**
 * @brief Interface for analog-to-digital converters.
 */
class IAdc {
public:
  virtual ~IAdc() = default;

  /// Read a single-ended channel. Returns a sample with the voltage and the
  /// acquisition timestamp.
  virtual std::optional<AdcSample> readSingleEnded() const = 0;
};

} // namespace msensor
