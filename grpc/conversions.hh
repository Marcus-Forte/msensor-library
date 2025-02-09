#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"

#include "gl_server.pb.h"
#include "sensors.pb.h"

msensor::Scan3DI fromGRPC(const sensors::PointCloud3 &msg);
msensor::IMUData fromGRPC(const sensors::IMUData &msg);

// To GL Server protobuff
gl::PointCloud3 toGRPC(const msensor::PointCloud3 &scan, float r, float g,
                       float b);