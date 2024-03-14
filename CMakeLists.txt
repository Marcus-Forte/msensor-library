cmake_minimum_required(VERSION 3.20)
project(rplidar_app)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(rplidar_sdk REQUIRED PATHS "C:\\workspace\\rplidar_sdk\\build\\install")

add_executable(app src/app.cc)
target_link_libraries(app rplidar_sdk::rplidar_sdk)