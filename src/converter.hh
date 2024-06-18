#pragma once

#include <vector>
#include <string>
#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "sl_lidar_cmd.h"

struct Point2
{
    float x;
    float y;
};
using PointCloud2 = std::vector<Point2>;

/// @brief Convert node data to pointcloud
/// @param nodes
/// @param count
/// @return
inline PointCloud2 toPointCloud(const sl_lidar_response_measurement_node_hq_t *nodes, int count)
{
    std::vector<Point2> points;
    points.reserve(count);
    int idx = 0;
    for (int pos = 0; pos < (int)count; ++pos)
    {
        if (nodes[pos].quality < 40)
            continue;
        
        float angle_in_pi = (nodes[pos].angle_z_q14 * M_PI_2) / 16384.f;
        const float dist_m = nodes[pos].dist_mm_q2 / 4000.0f;
        float x = -cos(angle_in_pi)*dist_m;
        float y = sin(angle_in_pi)*dist_m;
        Point2 pt;
        pt.x = x;
        pt.y = y;
        points.push_back(pt);
        // printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
        //        (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ",
        //        angle_in_pi,
        //        nodes[pos].dist_mm_q2 / 4.0f,
        //        nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    }

    return points;
}

inline void toFile(const PointCloud2 &pointcloud2, const std::string &filename = "points.txt")
{
    std::ofstream file(filename);
    for (const auto &pt : pointcloud2)
    {
        file << pt.x << " " << pt.y << " " << 0 << std::endl;
    }

    file.close();
}