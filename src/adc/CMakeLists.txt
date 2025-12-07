add_library(ads1115 STATIC ADS1115.cc)
target_include_directories(ads1115 PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(ads1115 PUBLIC IAdc)
