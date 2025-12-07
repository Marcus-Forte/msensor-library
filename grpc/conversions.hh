#pragma once

#include "interface/IImu.hh"
#include "interface/ILidar.hh"

#include "sensors.pb.h"

std::shared_ptr<msensor::Scan3DI> fromGRPC(const sensors::PointCloud3 &msg);
msensor::IMUData fromGRPC(const sensors::IMUData &msg);