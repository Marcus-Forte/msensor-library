
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

    add_library(grpc_publisher
    grpc_publisher.cc)
    target_link_libraries(grpc_publisher protolib)
    target_include_directories(grpc_publisher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endif()