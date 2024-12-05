link_libraries(sensors_proto ILidar Timing)

add_library(scan_recorder
ScanPlayer.cc
ScanRecorder.cc)

# add_executable(scan_checker
# ScanChecker.cc)

# install(TARGETS scan_checker 
# DESTINATION bin)