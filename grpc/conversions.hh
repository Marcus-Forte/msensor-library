#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"

#include "gl_server.pb.h"
#include "sensors.pb.h"

std::shared_ptr<msensor::Scan3DI> fromGRPC(const sensors::PointCloud3 &msg);
std::shared_ptr<msensor::IMUData> fromGRPC(const sensors::IMUData &msg);

// To GL Server protobuff
gl::PointCloud3 toGRPC(const std::shared_ptr<msensor::Scan3DI> &pi, float r,
                       float g, float b);