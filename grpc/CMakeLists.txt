# Sensor gRPC server

add_library(sensors_server
sensors_server.cc
sensors_service.cc
sensors_remote_client.cc)

target_link_libraries(sensors_server sensors_proto sensors_grpc conversions IImu ILidar Timing)
target_include_directories(sensors_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# OpenGL server grpc (https://github.com/Marcus-Forte/learning-opengl/blob/main/grpc/proto/gl_server.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/proto/gl_server.proto)
add_library(gl_server_grpc_client
${CMAKE_CURRENT_BINARY_DIR}/gl_server.pb.cc
${CMAKE_CURRENT_BINARY_DIR}/gl_server.grpc.pb.cc
)
target_include_directories(gl_server_grpc_client PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# Data conversions
add_library(conversions
conversions.cc)
target_link_libraries(conversions ILidar IImu sensors_proto gl_server_grpc_client)

# Proxy - forward sensor data to openGL server
add_executable(gl_proxy 
  gl_proxy.cc
)
target_link_libraries(gl_proxy sensors_grpc sensors_proto gl_server_grpc_client)
target_include_directories(gl_proxy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Publishes sensor data to GL server
add_library(gl_publisher
gl_publisher.cc
)
target_link_libraries(gl_publisher conversions)
