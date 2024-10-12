cmake_minimum_required(VERSION 3.20)
project(rplidar_app)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(format)

add_subdirectory(third_party/rplidar_sdk)
add_subdirectory(third_party/icm-20948)
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

add_executable(sensor_publisher src/sensor_publisher.cc)
target_link_libraries(sensor_publisher lidar grpc_server scan_recorder icm-20948-driver)

add_executable(main_test src/main_test.cc)
target_link_libraries(main_test rplidar_sdk::rplidar_sdk)

add_executable(scan_parser src/scan_parser.cc)
target_link_libraries(scan_parser lidar protolib)


set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install/${CMAKE_SYSTEM_PROCESSOR} CACHE PATH "default Install prefix" FORCE)

message(STATUS "installing to: " ${CMAKE_INSTALL_PREFIX})

install(TARGETS sensor_publisher gl_proxy scan_parser
RUNTIME DESTINATION bin)

