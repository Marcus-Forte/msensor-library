cmake_minimum_required(VERSION 3.20)
project(rplidar_app)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(third_party/rplidar_sdk)
add_subdirectory(grpc)


add_library(lidar 
  src/RPLidar.cc
  src/simLidar.cc)
target_include_directories(lidar PUBLIC include)
target_link_libraries(lidar rplidar_sdk::rplidar_sdk)

add_library(scan_recorder
src/ScanRecorder.cc)
target_link_libraries(scan_recorder protolib)
target_include_directories(scan_recorder PUBLIC include)

add_executable(rplidar_publisher src/rplidar_publisher.cc)
target_link_libraries(rplidar_publisher lidar grpc_server scan_recorder)

add_executable(main_test src/main_test.cc)
target_link_libraries(main_test rplidar_sdk::rplidar_sdk)

add_executable(scan_parser src/scan_parser.cc)
target_link_libraries(scan_parser lidar protolib)
# Format
find_program(formatter clang-format)
if(formatter)
  file(GLOB_RECURSE sourcefiles
    "include/*.hh"
    "src/*.cc"
    "grpc/*.cc"
    "grpc/*.hh")
  string (REPLACE ";" " " sourcefiles "${sourcefiles}")
  add_custom_target(format ALL
  COMMAND sh -c "clang-format -i ${sourcefiles}"
  VERBATIM)
endif()


set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install/${CMAKE_SYSTEM_PROCESSOR} CACHE PATH "default Install prefix" FORCE)

message(STATUS "installing to: " ${CMAKE_INSTALL_PREFIX})

install(TARGETS rplidar_publisher gl_proxy scan_parser
RUNTIME DESTINATION bin)