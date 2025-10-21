cmake_minimum_required(VERSION 3.20)
project(msensor_library)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(PCL 1.15 REQUIRED COMPONENTS common io) # for point types, point cloud serialization
link_libraries(${PCL_LIBRARIES})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(findGRPC)
include(generate_proto)

add_subdirectory(include)
add_subdirectory(third_party)
option (FORMAT_CODE "Format code" OFF)
option (IMPORT_ALL_RUNTIME "Import all runtime libraries, including system libraries like libc" OFF)
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

# Combines all
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME} Timing sensors_server scan_recorder sensors_proto gl_publisher)

add_executable(icm20948_publisher src/icm20948_publisher.cc)
target_link_libraries(icm20948_publisher ${PROJECT_NAME} icm-20948)

add_executable(rplidar_publisher src/rplidar_publisher.cc)
target_link_libraries(rplidar_publisher RPLidar ${PROJECT_NAME} icm-20948)

add_executable(mid360_publisher src/mid360_publisher.cc)
target_link_libraries(mid360_publisher mid360 ${PROJECT_NAME})

add_executable(sim_publisher src/sim_publisher.cc)
target_link_libraries(sim_publisher ${PROJECT_NAME} sim_lidar sim_imu)

include(GNUInstallDirs)
install(TARGETS rplidar_publisher mid360_publisher sim_publisher
DESTINATION ${CMAKE_INSTALL_BINDIR} RUNTIME_DEPENDENCY_SET runtime_targets)

# Experimental - install (all, including system libraries) runtime dependencies together for a standalone container.
# TODO Is there a better way?
if(IMPORT_ALL_RUNTIME)
  install(RUNTIME_DEPENDENCY_SET runtime_targets
  DESTINATION ${CMAKE_INSTALL_LIBDIR})
else()
# It is also possible to install only some of the libraries - given the system contains the others.
# The container must be based on the same distro
  install(RUNTIME_DEPENDENCY_SET runtime_targets PRE_EXCLUDE_REGEXES "." PRE_INCLUDE_REGEXES "pcl|boost|i2c|omp"
  DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cfg/mid360_config.json
DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()
