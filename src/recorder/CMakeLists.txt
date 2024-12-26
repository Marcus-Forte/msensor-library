link_libraries(sensors_proto ILidar Timing File)

add_library(scan_recorder
ScanPlayer.cc
ScanRecorder.cc)

add_executable(scan_checker
ScanChecker.cc)
target_link_libraries(scan_checker scan_recorder)

install(TARGETS scan_checker 
DESTINATION bin)