add_library(icm-20948
icm-20948.cc)
target_link_libraries(icm-20948 i2c IImu)

add_library(sim_imu
SimImu.cc)
target_link_libraries(sim_imu IImu)