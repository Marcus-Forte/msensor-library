#include "file/File.hh"
#include "recorder/ScanPlayer.hh"
#include "recorder/ScanRecorder.hh"

using namespace msensor;

int main(int argc, char **argv) {
  auto file = std::make_shared<File>();

  ScanRecorder recorder(file);

  recorder.start("test.pbscan");

  for (int i = 0; i < 100000; ++i) {

    auto scan = std::make_shared<Scan3DI>();

    scan->points->emplace_back(1, 2, 3, 4);
    auto imu = std::make_shared<IMUData>(1, 2, 3, 4, 5, 6);
    recorder.record(scan);
    recorder.record(imu);
  }

  ScanPlayer player("test.pbscan");
  // while (player.next()) {
  //   const auto entry = player.getLastEntry();
  //   std::cout << entry.entry_case() << std::endl;
  // }
}