# Sensor data proto lib.

# Generate the protobuf and gRPC code. Generated code will be named sensors.pb.cc and sensors.grpc.pb.cc
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/sensors.proto)

add_library(sensors_proto 
${CMAKE_CURRENT_BINARY_DIR}/sensors.pb.cc
)
target_include_directories(sensors_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(sensors_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(sensors_grpc
${CMAKE_CURRENT_BINARY_DIR}/sensors.grpc.pb.cc
)
target_include_directories(sensors_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(sensors_grpc ${_GRPC_GRPCPP})