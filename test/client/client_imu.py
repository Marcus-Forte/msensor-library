import grpc
from proto_gen import sensors_pb2_grpc
from google.protobuf.empty_pb2 import Empty

def main():
    with grpc.insecure_channel('192.168.3.232:50051') as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = Empty()
        stream = stub.getImu(request)
        last_time = 0
        for sample in stream:
            print(f"ax={sample.ax}, ay={sample.ay}, az={sample.az}, "
                  f"gx={sample.gx}, gy={sample.gy}, gz={sample.gz}, "
                  f"timestamp={sample.timestamp}")
            delta = sample.timestamp - last_time
            print(f"Δt = {delta} us")
            last_time = sample.timestamp

if __name__ == "__main__":
    main()