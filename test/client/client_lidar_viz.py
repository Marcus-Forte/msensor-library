import argparse

import grpc

import viser
import numpy as np

from proto_gen import sensors_pb2_grpc, sensors_pb2


DEFAULT_SERVER_ADDR = "localhost:50051"


def to_numpy(point_cloud: sensors_pb2.PointCloud3) -> np.ndarray:
    arr = np.zeros((len(point_cloud), 3), dtype=np.float32)
    for i, p in enumerate(point_cloud):
        arr[i, 0] = p.x
        arr[i, 1] = p.y
        arr[i, 2] = p.z
    return arr


def main():
    parser = argparse.ArgumentParser(description="Visualize LiDAR scans over gRPC.")
    parser.add_argument("--server", default=DEFAULT_SERVER_ADDR, help="gRPC target host:port")
    args = parser.parse_args()

    visa_server = viser.ViserServer()

    with grpc.insecure_channel(args.server) as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = sensors_pb2.SensorStreamRequest(queue_size=100)
        stream = stub.getScan(request)
        last = 0
        for scan in stream:
            print(f"Got points: {len(scan.points)} at {scan.timestamp}")
            deltaT = scan.timestamp - last
            print(f" DeltaT: {deltaT} ms")
            last = scan.timestamp

            visa_server.scene.add_point_cloud(
                name="/lidar",
                points=to_numpy(scan.points),
                colors=(0, 0, 0),
                point_size=0.05,
                point_shape="rounded",
            )


if __name__ == "__main__":
    main()
