cmake_minimum_required(VERSION 3.20)
project(msensor_library)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(PCL 1.14 REQUIRED) # for point types

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(findGRPC)
include(generate_proto)

add_subdirectory(include)
add_subdirectory(third_party)
option (FORMAT_CODE "Format code" OFF)
  if(FORMAT_CODE)
  include(format)
endif()

add_subdirectory(src/file)
add_subdirectory(src/timing)
add_subdirectory(src/lidar)
add_subdirectory(src/imu)
add_subdirectory(src/recorder)

add_subdirectory(proto)
add_subdirectory(grpc)

add_executable(rplidar_publisher src/rplidar_publisher.cc)
target_link_libraries(rplidar_publisher RPLidar sensors_server scan_recorder icm-20948)

add_executable(mid360_publisher src/mid360_publisher.cc)
target_link_libraries(mid360_publisher mid360 sensors_server scan_recorder)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install/ CACHE PATH "default Install prefix" FORCE)


include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()

message(STATUS "installing to: " ${CMAKE_INSTALL_PREFIX})

install(TARGETS rplidar_publisher
DESTINATION bin)

install(TARGETS mid360_publisher
DESTINATION bin)

