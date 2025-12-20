import argparse
import time

import grpc

from proto_gen import sensors_pb2, sensors_pb2_grpc


DEFAULT_SERVER_ADDR = "192.168.3.251:50054"
DEFAULT_CHANNEL = 0
DEFAULT_PERIOD_SEC = 0.5


def main():
	parser = argparse.ArgumentParser(description="Poll ADC samples over gRPC.")
	parser.add_argument("--server", default=DEFAULT_SERVER_ADDR, help="gRPC target host:port")
	parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL, help="ADC channel to poll")
	parser.add_argument("--period", type=float, default=DEFAULT_PERIOD_SEC, help="Seconds between polls")
	args = parser.parse_args()

	with grpc.insecure_channel(args.server) as channel:
		stub = sensors_pb2_grpc.SensorServiceStub(channel)
		request = sensors_pb2.AdcDataRequest(channel=args.channel)

		print(f"Polling ADC channel {args.channel} from {args.server} every {args.period}s. Press Ctrl+C to stop.")
		try:
			while True:
				try:
					resp = stub.GetAdc(request)
					print(f"ADC{args.channel}: {resp.sample:.4f} V @ {resp.timestamp}")
				except grpc.RpcError as e:
					print(f"gRPC error: {e.code().name} - {e.details()}")
				time.sleep(args.period)
		except KeyboardInterrupt:
			print("\nStopping ADC client.")


if __name__ == "__main__":
	main()
