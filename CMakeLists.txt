cmake_minimum_required(VERSION 3.20)
project(rplidar_app)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(USE_GRPC "Publish to default gRPC port" OFF)
# Fork from
find_package(rplidar_sdk REQUIRED PATHS "C:/workspace/rplidar_sdk/build/install")


add_executable(app src/app.cc)

target_link_libraries(app rplidar_sdk::rplidar_sdk)

if(USE_GRPC)
  add_subdirectory(grpc)
  target_link_libraries(app grpc_publisher)
  add_definitions(-DUSE_GRPC=1)
endif()