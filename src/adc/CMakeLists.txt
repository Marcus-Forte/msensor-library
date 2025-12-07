add_library(ads1115 STATIC ADS1115.cc)
target_link_libraries(ads1115 i2c IAdc)
