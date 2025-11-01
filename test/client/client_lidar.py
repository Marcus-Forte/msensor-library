import grpc
from proto_gen import sensors_pb2_grpc, sensors_pb2

def main():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = sensors_pb2.SensorStreamRequest(queue_size=100)
        stream = stub.getScan(request)
        for scan in stream:
            print(f"Got points: {len(scan.points)} at {scan.timestamp}")

if __name__ == "__main__":
    main()