# Sensor gRPC server

add_library(grpc_sensor_server
grpc_server.cc
get_points_service.cc)
target_link_libraries(grpc_sensor_server sensors_grpc IImu ILidar)
target_include_directories(grpc_sensor_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Proxy - forward to openGL server (https://github.com/Marcus-Forte/learning-opengl/blob/main/grpc/proto/points.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/proto/opengl_srv_points.proto)
add_executable(gl_proxy 
  gl_proxy.cc
  ${CMAKE_CURRENT_BINARY_DIR}/opengl_srv_points.pb.cc
  ${CMAKE_CURRENT_BINARY_DIR}/opengl_srv_points.grpc.pb.cc
)
target_link_libraries(gl_proxy sensors_grpc sensors_proto)
target_include_directories(gl_proxy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS gl_proxy
DESTINATION bin)
