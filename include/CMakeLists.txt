include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(IFile INTERFACE)
target_include_directories(IFile INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(IImu INTERFACE)
target_include_directories(IImu INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IImu INTERFACE Timing)

add_library(ILidar INTERFACE)
target_include_directories(ILidar INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IImu INTERFACE Timing)

# Lidar drivers
add_library(IRPlidar INTERFACE)
target_include_directories(IRPlidar INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IRPlidar INTERFACE rplidar_sdk ILidar)

