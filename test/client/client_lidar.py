import argparse

import grpc
from proto_gen import sensors_pb2_grpc, sensors_pb2


DEFAULT_SERVER_ADDR = "192.168.3.251:50053"


def main():
    parser = argparse.ArgumentParser(description="Stream LiDAR scans over gRPC.")
    parser.add_argument("--server", default=DEFAULT_SERVER_ADDR, help="gRPC target host:port")
    args = parser.parse_args()

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


if __name__ == "__main__":
    main()