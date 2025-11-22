import grpc
from proto_gen import sensors_pb2_grpc, sensors_pb2

def main():
    with grpc.insecure_channel('192.168.3.251:50052') as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = sensors_pb2.SensorStreamRequest(queue_size=1)
        stream = stub.getImu(request)
        lastT = 0
        for imu in stream:
            # print(f"Got imu: {imu}")
            deltaT = imu.timestamp - lastT
            print(f" DeltaT: {deltaT} ms")
            lastT = imu.timestamp

if __name__ == "__main__":
    main()