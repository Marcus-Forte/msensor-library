#include "lidar/ILidar.hh"
#include "sensors_remote_client.hh"
#include "sensors_server.hh"
#include <chrono>
#include <gtest/gtest.h>

class TestServer : public ::testing::Test {
public:
  void SetUp() override {
    server = std::make_shared<SensorsServer>();
    client = std::make_shared<SensorsRemoteClient>("localhost:50051");
  }

protected:
  std::shared_ptr<SensorsServer> server;
  std::shared_ptr<SensorsRemoteClient> client;
};

// TODO fix infinite loop
TEST_F(TestServer, DISABLED_TestServer) {

  server->start();
  client->start();

  auto scan = std::shared_ptr<msensor::Scan3DI>();
  scan->points->emplace_back(1, 2, 3);
  scan->timestamp = 10;

  auto imu = std::make_shared<msensor::IMUData>(1, 2, 3, 4, 5, 6, 7);

  server->publishScan(scan);
  server->publishImu(imu);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(500)); // Wait for the data to reach the client.
  const auto scan_read = client->getScan();
  const auto &scan_points = *scan_read->points;
  const auto imu_read = client->getImuData();

  EXPECT_EQ(scan_read->timestamp, 10);
  ASSERT_GE(scan_read->points->size(), 1);
  EXPECT_EQ(scan_points[0].x, 1);
  EXPECT_EQ(scan_points[0].y, 2);
  EXPECT_EQ(scan_points[0].z, 3);

  EXPECT_EQ(imu_read->ax, 1);
  EXPECT_EQ(imu_read->ay, 2);
  EXPECT_EQ(imu_read->az, 3);
  EXPECT_EQ(imu_read->gx, 4);
  EXPECT_EQ(imu_read->gy, 5);
  EXPECT_EQ(imu_read->gz, 6);

  client->stop();
  server->stop();
}