cmake_minimum_required(VERSION 3.20)
project(rplidar_app)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(format)

include_directories(include)

add_subdirectory(third_party/rplidar_sdk)
add_subdirectory(src/lidar)
add_subdirectory(src/imu)
add_subdirectory(src/recorder)

add_subdirectory(proto)
add_subdirectory(grpc)

add_executable(sensor_publisher src/sensor_publisher.cc)
target_link_libraries(sensor_publisher lidar grpc_sensor_server scan_recorder icm-20948)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install/${CMAKE_SYSTEM_PROCESSOR} CACHE PATH "default Install prefix" FORCE)

message(STATUS "installing to: " ${CMAKE_INSTALL_PREFIX})

install(TARGETS sensor_publisher gl_proxy
RUNTIME DESTINATION bin)

