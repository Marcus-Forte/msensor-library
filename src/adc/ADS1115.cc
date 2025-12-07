#include "adc/ADS1115.hh"

#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "timing/timing.hh"

#include <optional>
#include <stdexcept>
#include <string>

namespace msensor {

inline uint8_t ADS1115::write_(uint8_t reg_address, uint8_t data) const {
  return i2c_smbus_write_byte_data(i2c_device_fd_, reg_address, data);
}

inline uint8_t ADS1115::read_(uint8_t reg_address) const {
  return i2c_smbus_read_byte_data(i2c_device_fd_, reg_address);
}

inline int16_t ADS1115::read_word_(uint8_t reg_address) const {
  return i2c_smbus_read_word_data(i2c_device_fd_, reg_address);
}

namespace {
constexpr uint8_t REG_CONVERSION = 0x00;
constexpr uint8_t REG_CONFIG = 0x01;
constexpr uint16_t CONFIG_OS_SINGLE = 0x8000;
constexpr uint16_t CONFIG_MODE_SINGLE = 0x0100;
constexpr uint16_t CONFIG_COMP_QUEUE_DISABLE = 0x0003;
} // namespace

uint16_t ADS1115::gainBits(Gain gain) {
  return (static_cast<uint16_t>(gain) & 0x7) << 9;
}

double ADS1115::gainRange(Gain gain) {
  switch (gain) {
  case Gain::PLUS_MINUS_6_144:
    return 6.144;
  case Gain::PLUS_MINUS_4_096:
    return 4.096;
  case Gain::PLUS_MINUS_2_048:
    return 2.048;
  case Gain::PLUS_MINUS_1_024:
    return 1.024;
  case Gain::PLUS_MINUS_0_512:
    return 0.512;
  case Gain::PLUS_MINUS_0_256:
    return 0.256;
  }
  return 2.048;
}

ADS1115::ADS1115(int i2c_device, uint8_t i2c_ads_address)
    : i2c_device_(i2c_device), i2c_ads_address_(i2c_ads_address) {

  const std::string i2c_device_file = "/dev/i2c-" + std::to_string(i2c_device);
  i2c_device_fd_ = open(i2c_device_file.c_str(), O_RDWR);

  if (i2c_device_fd_ < 0) {
    throw std::runtime_error("unable to open I2C (system) device: " +
                             std::to_string(i2c_device));
  }

  if (ioctl(i2c_device_fd_, I2C_SLAVE, i2c_ads_address_) < 0) {
    throw std::runtime_error("unable to open IMU device: " +
                             std::to_string(i2c_ads_address_)); // to hex?
  }
}

std::optional<AdcSample> ADS1115::readSingleEnded(AdcChannel channel) {
  auto raw = readConversion();
  if (!raw) {
    return std::nullopt;
  }
  return AdcSample{convertRawToVoltage(*raw), timing::getNowUs()};
}

bool ADS1115::init(Gain gain, DataRate rate, Channel channel) {
  gain_ = gain;
  dataRate_ = rate;

  uint16_t config = CONFIG_OS_SINGLE;
  config |= (static_cast<uint16_t>(channel) << 12);
  config |= gainBits(gain_);
  config |= dataRateBits(dataRate_);
  config |= CONFIG_MODE_SINGLE;
  config |= CONFIG_COMP_QUEUE_DISABLE;
  return write_(REG_CONFIG, config);
}

std::optional<int16_t> ADS1115::readConversion() {
  const auto raw = read_word_(REG_CONVERSION);
  if (!raw) {
    return std::nullopt;
  }
  return raw;
}

double ADS1115::convertRawToVoltage(int16_t raw) const {
  double range = gainRange(gain_);
  return (static_cast<double>(raw) / 32768.0) * range;
}

} // namespace msensor
