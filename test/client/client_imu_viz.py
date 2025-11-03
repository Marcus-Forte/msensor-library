import grpc

# gRPC stubs for your SENSOR server
from proto_gen import sensors_pb2_grpc, sensors_pb2, plot_pb2_grpc, plot_pb2

# --- Configuration ---
SENSOR_SERVER_ADDR = '192.168.3.232:50051'
PLOT_SERVER_ADDR = 'host.docker.internal:50052' # Assuming it's on the same machine

# --- Plot Configuration ---

# We'll create 6 signals and map them. Must match the configured signal
# https://github.com/Marcus-Forte/msensor-remote-plotter/blob/main/config/config_imu_signals.py

SIGNAL_ID_ACC_X = 10
SIGNAL_ID_ACC_Y = 11
SIGNAL_ID_ACC_Z = 12
SIGNAL_ID_GYRO_X = 20
SIGNAL_ID_GYRO_Y = 21
SIGNAL_ID_GYRO_Z = 22

def stream_adapter(sensor_stub):
    """
    Generator function.
    1. Gets a sample from the SENSOR stream.
    2. Transforms it into a StreamPointsBatch.
    3. Yields the batch to be sent to the PLOT stream.
    """
    
    # Start the sensor stream
    request = sensors_pb2.SensorStreamRequest(queue_size=100)
    try:
        sensor_stream = sensor_stub.getImu(request)
        print("Connected to sensor stream. Forwarding data...")

        # Loop forever, getting data from the sensor
        for sample in sensor_stream:
            # (This assumes your sensor 'sample' has fields ax, ay, az, gx, gy, gz)
            print(sample)
            # Create a batch with all 6 data points
            batch = plot_pb2.streamPointRequest(
                points=[
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_X, value=sample.ax),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_Y, value=sample.ay),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_Z, value=sample.az),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_X, value=sample.gx),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_Y, value=sample.gy),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_Z, value=sample.gz),
                ]
            )
            
            # Yield this batch to the plot server stream
            yield batch
            
    except grpc.RpcError as e:
        print(f"!!! ERROR with sensor stream: {e.details()} !!!")
        return # This will end the generator and close the stream to the plot server

def main():
    # Open two channels simultaneously
    with grpc.insecure_channel(SENSOR_SERVER_ADDR) as sensor_channel, \
         grpc.insecure_channel(PLOT_SERVER_ADDR) as plot_channel:
        
        # Create stubs for both servers
        sensor_stub = sensors_pb2_grpc.SensorServiceStub(sensor_channel)
        plot_stub = plot_pb2_grpc.PlotServiceStub(plot_channel)

        try:
            print(f"Connected to plot server at {PLOT_SERVER_ADDR}")
        except grpc.FutureTimeoutError:
            print(f"!!! FAILED to connect to plot server at {PLOT_SERVER_ADDR} !!!")
            return


        try:
            print(f"Connected to sensor server at {SENSOR_SERVER_ADDR}")
        except grpc.FutureTimeoutError:
            print(f"!!! FAILED to connect to sensor server at {SENSOR_SERVER_ADDR} !!!")
            return

        # Start the streaming
        # This one line is the core logic:
        # It calls plot_stub.streamPlot() and passes it a generator.
        # The generator (stream_adapter) gets data from sensor_stub.
        # gRPC handles the streaming between the two.
        print("Starting stream... Press Ctrl+C to stop.")
        try:
            plot_stub.streamPlot(stream_adapter(sensor_stub))
            
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.UNAVAILABLE:
                print("Plot server disconnected. Exiting.")
            else:
                print(f"An error occurred: {e}")
        except KeyboardInterrupt:
            print("\nStopping bridge client.")


if __name__ == "__main__":
    main()

