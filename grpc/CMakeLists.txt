
# Add .cmake as modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(generate_proto)
include(findGRPC)

if(gRPC_FOUND)
    message(STATUS "gRPC detected.")
    generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/proto/points.proto)
    add_library(protolib 
    ${CMAKE_CURRENT_BINARY_DIR}/points.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/points.grpc.pb.cc)
    target_include_directories(protolib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(protolib ${_GRPC_GRPCPP})

    add_library(grpc_server
    grpc_server.cc
    get_points_service.cc)
    target_link_libraries(grpc_server protolib)
    target_include_directories(grpc_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    # Proxy
    generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/proto/opengl_srv_points.proto)
    add_executable(gl_proxy 
    gl_proxy.cc
    ${CMAKE_CURRENT_BINARY_DIR}/opengl_srv_points.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/opengl_srv_points.grpc.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/points.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/points.grpc.pb.cc)
    target_link_libraries(gl_proxy ${_GRPC_GRPCPP})
    target_include_directories(gl_proxy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
endif()