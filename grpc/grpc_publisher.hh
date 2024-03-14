#pragma once

#include "proto/points.grpc.pb.h"
#include "proto/points.pb.h"
#include <grpcpp/grpcpp.h>

class gRPCPublisher
{
public:
    gRPCPublisher(const std::string &url);

    template <class T>
    void send(const std::vector<T> &container) const
    {
        auto channel_state = channel_->GetState(false);

        if (channel_state == GRPC_CHANNEL_TRANSIENT_FAILURE ||  channel_state == GRPC_CHANNEL_SHUTDOWN)
        {
            std::cerr << "Unable to publish!" << std::endl;
            return;
        }
        PointCloud3 cloud;
        cloud.set_entity_name("rplidar");
        for (const auto &el : container)
        {
            auto pt = cloud.add_points();
            pt->set_x(el.x);
            pt->set_y(el.y);
            // 2D Lidar
            pt->set_z(0.0);
            pt->set_r(0.0);
            pt->set_g(1.0);
            pt->set_b(0.0);
        }

        grpc::ClientContext context;
        google::protobuf::Empty response;
        stub_->addPointCloud(&context, cloud, &response);
    }

private:
    std::unique_ptr<addToScene::Stub> stub_;
    std::shared_ptr<grpc::Channel> channel_;
};