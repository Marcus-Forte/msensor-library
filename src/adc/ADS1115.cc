#include "adc/ADS1115.hh"

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "timing/timing.hh"

#include <chrono>
#include <cstring>
#include <optional>
#include <thread>

namespace msensor {

namespace {
constexpr uint8_t REG_CONVERSION = 0x00;
constexpr uint8_t REG_CONFIG = 0x01;
constexpr uint16_t CONFIG_OS_SINGLE = 0x8000;
constexpr uint16_t CONFIG_MODE_SINGLE = 0x0100;
constexpr uint16_t CONFIG_COMP_QUEUE_DISABLE = 0x0003;
} // namespace

ADS1115::Multiplexer ADS1115::singleMux(AdcChannel channel) {
  using Multiplexer = ADS1115::Multiplexer;
  switch (channel) {
  case AdcChannel::CHANNEL_0:
    return Multiplexer::SINGLE_0;
  case AdcChannel::CHANNEL_1:
    return Multiplexer::SINGLE_1;
  case AdcChannel::CHANNEL_2:
    return Multiplexer::SINGLE_2;
  case AdcChannel::CHANNEL_3:
    return Multiplexer::SINGLE_3;
  default:
    return Multiplexer::SINGLE_0;
  }
}

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

uint16_t ADS1115::dataRateBits(DataRate rate) {
  return (static_cast<uint16_t>(rate) & 0x7) << 5;
}

ADS1115::ADS1115(int i2cBus, uint8_t address)
    : fileDescriptor_(-1), i2cBus_(i2cBus), address_(address) {}

ADS1115::~ADS1115() { close(); }

bool ADS1115::open() {
  if (fileDescriptor_ >= 0) {
    return true;
  }

  std::string path = "/dev/i2c-" + std::to_string(i2cBus_);
  fileDescriptor_ = ::open(path.c_str(), O_RDWR);
  if (fileDescriptor_ < 0) {
    return false;
  }

  if (::ioctl(fileDescriptor_, I2C_SLAVE, address_) < 0) {
    close();
    return false;
  }

  return true;
}

void ADS1115::close() {
  if (fileDescriptor_ >= 0) {
    ::close(fileDescriptor_);
    fileDescriptor_ = -1;
  }
}

bool ADS1115::isReady() const { return fileDescriptor_ >= 0; }

void ADS1115::setGain(Gain gain) { gain_ = gain; }

void ADS1115::setDataRate(DataRate rate) { dataRate_ = rate; }

std::optional<AdcSample> ADS1115::readSingleEnded(AdcChannel channel) {
  if (!isReady() && !open()) {
    return std::nullopt;
  }
  Multiplexer mux = singleMux(channel);
  if (!configureAndStart(mux)) {
    return std::nullopt;
  }
  auto raw = readConversion();
  if (!raw) {
    return std::nullopt;
  }
  return AdcSample{convertRawToVoltage(*raw), timing::getNowUs()};
}

bool ADS1115::configureAndStart(Multiplexer mux) {
  uint16_t config = CONFIG_OS_SINGLE;
  config |= (static_cast<uint16_t>(mux) << 12);
  config |= gainBits(gain_);
  config |= dataRateBits(dataRate_);
  config |= CONFIG_MODE_SINGLE;
  config |= CONFIG_COMP_QUEUE_DISABLE;
  return writeRegister(REG_CONFIG, config) && waitForConversionComplete();
}

bool ADS1115::waitForConversionComplete() {
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  while (std::chrono::steady_clock::now() < deadline) {
    auto config = readRegister(REG_CONFIG);
    if (!config) {
      return false;
    }
    if ((*config & CONFIG_OS_SINGLE) != 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return false;
}

static uint16_t hostToBe(uint16_t value) {
  return static_cast<uint16_t>((value >> 8) | (value << 8));
}

bool ADS1115::writeRegister(uint8_t reg, uint16_t value) {
  if (fileDescriptor_ < 0) {
    return false;
  }
  return i2c_smbus_write_word_data(fileDescriptor_, reg, hostToBe(value)) == 0;
}

std::optional<uint16_t> ADS1115::readRegister(uint8_t reg) {
  if (fileDescriptor_ < 0) {
    return std::nullopt;
  }
  auto raw = i2c_smbus_read_word_data(fileDescriptor_, reg);
  if (raw < 0) {
    return std::nullopt;
  }
  return hostToBe(static_cast<uint16_t>(raw));
}

std::optional<int16_t> ADS1115::readConversion() {
  auto raw = readRegister(REG_CONVERSION);
  if (!raw) {
    return std::nullopt;
  }
  return static_cast<int16_t>(*raw);
}

double ADS1115::convertRawToVoltage(int16_t raw) const {
  double range = gainRange(gain_);
  return (static_cast<double>(raw) / 32768.0) * range;
}

} // namespace msensor
