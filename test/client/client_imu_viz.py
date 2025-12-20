import argparse
import grpc
import math
# gRPC stubs for your SENSOR server
from proto_gen import sensors_pb2_grpc, sensors_pb2, plot_pb2_grpc, plot_pb2
from google.protobuf.empty_pb2 import Empty

# --- Configuration ---
DEFAULT_SENSOR_SERVER_ADDR = '192.168.3.251:50052'
DEFAULT_PLOT_SERVER_ADDR = 'localhost:50052'  # Assuming it's on the same machine

# --- Plot Configuration ---

# We'll create 6 signals and map them. Must match the configured signal
# https://github.com/Marcus-Forte/msensor-remote-plotter/blob/main/config/config_imu_signals.py

"""
@startuml

object "IMU Sensor Server" as imu {
+ getImu()
}
object "Plot Server" as plot
object "Client" as client {
+ fuse_data()
}

client --> imu : streams from
client --> plot : streams to

@enduml
"""


AXIS_ACC_ID = 3
AXIS_GYRO_ID = 4

SIGNAL_ID_ACC_X = 10
SIGNAL_ID_ACC_Y = 11
SIGNAL_ID_ACC_Z = 12

SIGNAL_ID_GYRO_X = 20
SIGNAL_ID_GYRO_Y = 21
SIGNAL_ID_GYRO_Z = 22

SIGNAL_ID_ACC_PITCH = 30
SIGNAL_ID_ACC_ROLL = 31
SIGNAL_ID_GYRO_PITCH = 40
SIGNAL_ID_GYRO_ROLL = 41
SIGNAL_ID_FUSED_PITCH = 50
SIGNAL_ID_FUSED_ROLL = 51

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

        dt = 0
        last_timestamp = 0.0

        # Process each sample from the sensor stream
        gyro_pitch = 0.0
        gyro_roll = 0.0
        gyro_yaw = 0.0

        fused_roll = 0.0
        fused_pitch = 0.0
        for sample in sensor_stream:
            # Filter sample time difference
            dt = (sample.timestamp - last_timestamp) / 1000000 # in seconds
            if(dt < 0 or dt > 1):
                dt = 0.01  # Default to 10ms if invalid
            last_timestamp = sample.timestamp

            # Let's do the experiment here.
            # Accelerometer-based angle estimation
            roll_angle_acc = math.atan2(sample.ay, sample.az) * (180.0 / math.pi)
            pitch_angle_acc = math.atan2(-sample.ax, math.sqrt(sample.ay**2 + sample.az**2)) * (180.0 / math.pi)
            

            # Gyroscope-based angle estimation (not used in this example)
            gyro_roll += sample.gx * dt * (180.0 / math.pi)
            gyro_pitch += sample.gy * dt * (180.0 / math.pi)
            gyro_yaw += sample.gz * dt * (180.0 / math.pi)
            
            # # Simple complementary filter for fused angle estimation
            # alpha = 0.98 # Closer to 1 means more gyro, closer to 0 means more acc
            # fused_roll = alpha * (fused_roll + sample.gx * dt * (180.0 / math.pi)) + (1 - alpha) * roll_angle_acc
            # fused_pitch = alpha * (fused_pitch + sample.gy * dt * (180.0 / math.pi)) + (1 - alpha) * pitch_angle_acc

            # Send to the plot server
            batch = plot_pb2.streamPointRequest(
                points=[
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_X, value=sample.ax),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_Y, value=sample.ay),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_Z, value=sample.az),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_X, value=sample.gx),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_Y, value=sample.gy),
                    plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_Z, value=sample.gz),
                    # plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_ROLL, value=roll_angle_acc),
                    # plot_pb2.streamPoint(signal_id=SIGNAL_ID_ACC_PITCH, value=pitch_angle_acc),
                    # plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_ROLL, value=gyro_roll),
                    # plot_pb2.streamPoint(signal_id=SIGNAL_ID_GYRO_PITCH, value=gyro_pitch),
                    # plot_pb2.streamPoint(signal_id=SIGNAL_ID_FUSED_ROLL, value=fused_roll),
                ]
            )

            
            
            # Yield this batch to the plot server stream
            yield batch
            
    except grpc.RpcError as e:
        print(f"!!! ERROR with sensor stream: {e.details()} !!!")
        return # This will end the generator and close the stream to the plot server

def main():
    parser = argparse.ArgumentParser(description="Bridge IMU stream to plotting service.")
    parser.add_argument("--sensor", default=DEFAULT_SENSOR_SERVER_ADDR, help="Sensor gRPC host:port")
    parser.add_argument("--plot", default=DEFAULT_PLOT_SERVER_ADDR, help="Plot gRPC host:port")
    args = parser.parse_args()

    # Open two channels simultaneously
    with grpc.insecure_channel(args.sensor) as sensor_channel, \
         grpc.insecure_channel(args.plot) as plot_channel:
        
        sensor_stub = sensors_pb2_grpc.SensorServiceStub(sensor_channel)
        plot_stub = plot_pb2_grpc.PlotServiceStub(plot_channel)

        # Plot configuration
        plot_stub.clearAll(Empty())
        plot_stub.AddAxis(plot_pb2.AddAxisRequest(axis_id=AXIS_ACC_ID, number_of_samples=1000, plot_title="raw acc"))
        plot_stub.AddAxis(plot_pb2.AddAxisRequest(axis_id=AXIS_GYRO_ID, number_of_samples=1000, plot_title="raw gyro"))
   
        plot_stub.AddSignal(plot_pb2.AddSignalRequest(signal_id=SIGNAL_ID_ACC_X, axis_id=AXIS_ACC_ID, signal_name="Acc X"))
        plot_stub.AddSignal(plot_pb2.AddSignalRequest(signal_id=SIGNAL_ID_ACC_Y, axis_id=AXIS_ACC_ID, signal_name="Acc Y"))
        plot_stub.AddSignal(plot_pb2.AddSignalRequest(signal_id=SIGNAL_ID_ACC_Z, axis_id=AXIS_ACC_ID, signal_name="Acc Z"))


        plot_stub.AddSignal(plot_pb2.AddSignalRequest(signal_id=SIGNAL_ID_GYRO_X, axis_id=AXIS_GYRO_ID, signal_name="Gyro X"))
        plot_stub.AddSignal(plot_pb2.AddSignalRequest(signal_id=SIGNAL_ID_GYRO_Y, axis_id=AXIS_GYRO_ID, signal_name="Gyro Y"))
        plot_stub.AddSignal(plot_pb2.AddSignalRequest(signal_id=SIGNAL_ID_GYRO_Z, axis_id=AXIS_GYRO_ID, signal_name="Gyro Z"))

        try:
            print(f"Connected to plot server at {args.plot}")
        except grpc.FutureTimeoutError:
            print(f"!!! FAILED to connect to plot server at {args.plot} !!!")
            return


        try:
            print(f"Connected to sensor server at {args.sensor}")
        except grpc.FutureTimeoutError:
            print(f"!!! FAILED to connect to sensor server at {args.sensor} !!!")
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

