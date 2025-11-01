import grpc
from proto_gen import sensors_pb2_grpc, sensors_pb2

def main():
    with grpc.insecure_channel('192.168.3.232:50051') as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = sensors_pb2.SensorStreamRequest(queue_size=100)
        stream = stub.getImu(request)
        for imu in stream:
            print(f"Got imu: {imu}")

if __name__ == "__main__":
    main()