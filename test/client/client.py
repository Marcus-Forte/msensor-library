import grpc
from proto_gen import sensors_pb2, sensors_pb2_grpc
from google.protobuf.empty_pb2 import Empty

def main():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = Empty()
        stream = stub.getScan(request)
        for scan in stream:
            print(f"Got points: {len(scan.points)} at {scan.timestamp}")

if __name__ == "__main__":
    main()