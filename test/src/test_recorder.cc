#include "IFileMock.hh"
#include "recorder/ScanRecorder.hh"
#include <gtest/gtest.h>

using namespace testing;

class TestRecorder : public ::testing::Test {
public:
  void SetUp() override {
    file_mock_ = std::make_shared<testing::StrictMock<IFileMock>>();
    recorder_ = std::make_shared<ScanRecorder>(file_mock_);

    EXPECT_CALL(*file_mock_, close()); // from destructor
  }

protected:
  std::shared_ptr<ScanRecorder> recorder_;
  std::shared_ptr<testing::StrictMock<IFileMock>> file_mock_;
};

TEST_F(TestRecorder, start) {
  EXPECT_CALL(*file_mock_, open(testing::_));

  recorder_->start();
}

TEST_F(TestRecorder, record_scan) {
  EXPECT_CALL(*file_mock_, open(testing::_));
  EXPECT_CALL(*file_mock_, write(_, _));

  // redirect to string stream
  std::stringstream stream;
  EXPECT_CALL(*file_mock_, ostream()).WillRepeatedly(Return(&stream));
  Scan3D scan;
  scan.points.emplace_back(1, 2, 3);
  scan.points.emplace_back(1, 2, 3);

  recorder_->start();
  recorder_->record(scan);

  EXPECT_EQ(stream.str().size(), 44);
}

TEST_F(TestRecorder, record_imu) {
  EXPECT_CALL(*file_mock_, open(testing::_));
  EXPECT_CALL(*file_mock_, write(_, _));

  // redirect to string stream
  std::stringstream stream;
  EXPECT_CALL(*file_mock_, ostream()).WillRepeatedly(Return(&stream));
  IMUData imu;

  recorder_->start();
  recorder_->record(imu);

  EXPECT_EQ(stream.str().size(),
            34); // struct: 32 bytes. 2 bytes from protobuf.
}