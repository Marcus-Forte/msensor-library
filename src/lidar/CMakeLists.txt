link_libraries(ILidar)

add_library(sim_lidar
SimLidar.cc)

add_library(mid360
mid360.cc)
target_link_libraries(mid360 livox_lidar_sdk_static)

add_library(RPLidar 
  RPLidar.cc)
target_link_libraries(RPLidar rplidar_sdk ILidar)