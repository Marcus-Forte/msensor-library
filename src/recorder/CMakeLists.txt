link_libraries(sensors_proto ILidar)

add_library(scan_recorder
ScanRecorder.cc)

add_executable(scan_checker
ScanChecker.cc)

install(TARGETS scan_checker 
DESTINATION bin)