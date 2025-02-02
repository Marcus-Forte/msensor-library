#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"

#include "sensors.pb.h"

msensor::Scan3DI fromGRPC(const sensors::PointCloud3 &msg);
msensor::IMUData fromGRPC(const sensors::IMUData &msg);
