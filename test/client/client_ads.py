import time

import grpc

from proto_gen import sensors_pb2, sensors_pb2_grpc


SERVER_ADDR = "192.168.3.251:50054"  # adjust if your server runs elsewhere
CHANNEL = 0  # ADC channel to poll
PERIOD_SEC = 0.5

def main():
	with grpc.insecure_channel(SERVER_ADDR) as channel:
		stub = sensors_pb2_grpc.SensorServiceStub(channel)
		request = sensors_pb2.AdcDataRequest(channel=CHANNEL)

		print(f"Polling ADC channel {CHANNEL} from {SERVER_ADDR} every {PERIOD_SEC}s. Press Ctrl+C to stop.")
		try:
			while True:
				try:
					resp = stub.GetAdc(request)
					print(f"ADC{CHANNEL}: {resp.sample:.4f} V @ {resp.timestamp}")
				except grpc.RpcError as e:
					print(f"gRPC error: {e.code().name} - {e.details()}")
				time.sleep(PERIOD_SEC)
		except KeyboardInterrupt:
			print("\nStopping ADC client.")


if __name__ == "__main__":
	main()
