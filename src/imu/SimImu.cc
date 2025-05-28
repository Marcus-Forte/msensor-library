#include "imu/SimImu.hh"
#include "timing/timing.hh"
#include <random>
#include <thread>

namespace msensor {

std::shared_ptr<IMUData> SimImu::getImuData() {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> dis(-1.0, 1.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz.
  auto data = std::make_shared<IMUData>();

  data->ax = dis(gen);
  data->ay = dis(gen);
  data->az = dis(gen);
  data->gx = dis(gen);
  data->gy = dis(gen);
  data->gz = dis(gen);
  data->timestamp = timing::getNowUs();
  return data;
}

} // namespace msensor