import argparse

import grpc
from proto_gen import sensors_pb2_grpc, sensors_pb2


DEFAULT_SERVER_ADDR = "192.168.3.251:50052"


def main():
    parser = argparse.ArgumentParser(description="Stream IMU data over gRPC.")
    parser.add_argument("--server", default=DEFAULT_SERVER_ADDR, help="gRPC target host:port")
    args = parser.parse_args()

    with grpc.insecure_channel(args.server) as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = sensors_pb2.SensorStreamRequest(queue_size=1)
        stream = stub.getImu(request)
        lastT = 0
        for imu in stream:
            deltaT = imu.timestamp - lastT
            print(f" DeltaT: {deltaT} ms")
            lastT = imu.timestamp


if __name__ == "__main__":
    main()
