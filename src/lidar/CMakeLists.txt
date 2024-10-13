add_library(simLidar
simLidar.cc)

add_library(lidar 
  RPLidar.cc
  simLidar.cc)
target_link_libraries(lidar rplidar_sdk::rplidar_sdk)