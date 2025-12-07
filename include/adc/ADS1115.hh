#pragma once

#include "interface/IAdc.hh"

#include <cstdint>
#include <optional>

namespace msensor {

/// Minimal ADS1115 driver backing the generic ADC interface.
class ADS1115 final : public IAdc {
public:
  enum class Gain : uint16_t {
    PLUS_MINUS_6_144 = 0,
    PLUS_MINUS_4_096,
    PLUS_MINUS_2_048,
    PLUS_MINUS_1_024,
    PLUS_MINUS_0_512,
    PLUS_MINUS_0_256,
  };

  enum class DataRate : uint16_t {
    SPS_8 = 0,
    SPS_16,
    SPS_32,
    SPS_64,
    SPS_128,
    SPS_250,
    SPS_475,
    SPS_860,
  };

  ADS1115(int i2cBus = 1, uint8_t address = 0x48);
  ~ADS1115() override;

  bool open();
  void close();
  bool isReady() const;

  void setGain(Gain gain);
  void setDataRate(DataRate rate);

  std::optional<AdcSample> readSingleEnded(AdcChannel channel) override;

private:
  enum class Multiplexer : uint16_t {
    SINGLE_0 = 4,
    SINGLE_1 = 5,
    SINGLE_2 = 6,
    SINGLE_3 = 7,
  };

  static Multiplexer singleMux(AdcChannel channel);
  static uint16_t gainBits(Gain gain);
  static double gainRange(Gain gain);
  static uint16_t dataRateBits(DataRate rate);

  bool configureAndStart(Multiplexer mux);
  bool waitForConversionComplete();
  bool writeRegister(uint8_t reg, uint16_t value);
  std::optional<uint16_t> readRegister(uint8_t reg);
  std::optional<int16_t> readConversion();
  double convertRawToVoltage(int16_t raw) const;

  int fileDescriptor_;
  int i2cBus_;
  uint8_t address_;
  Gain gain_{Gain::PLUS_MINUS_2_048};
  DataRate dataRate_{DataRate::SPS_128};
};

} // namespace msensor
