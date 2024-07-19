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
add_executable(rplidar_publisher src/rplidar_publisher.cc)
target_link_libraries(rplidar_publisher lidar grpc_publisher)


add_executable(main_test src/main_test.cc)
target_link_libraries(main_test rplidar_sdk::rplidar_sdk)

# Format
find_program(formatter clang-format)
if(formatter)
  file(GLOB_RECURSE sourcefiles
    "include/*.hh"
    "src/*.cc")
  string (REPLACE ";" " " sourcefiles "${sourcefiles}")
  add_custom_target(format ALL
  COMMAND sh -c "clang-format -i ${sourcefiles}"
  VERBATIM)
endif()

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR} CACHE PATH "default Install prefix" FORCE)
endif()

install(TARGETS rplidar_publisher
# RUNTIME_DEPENDENCIES
# POST_EXCLUDE_REGEXES ".*ld-linux.*"  # Linux dynamic linker
#                     "libc.so"      # Common C libraries
#                     "libgcc_"
#                     "libstdc"
#                     "libm.*"      # Math library
# LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)


set(CPACK_PACKAGE_NAME "rplidar")
set(CPACK_GENERATOR "RPM")
include(CPack)