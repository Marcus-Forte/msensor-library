import grpc
from proto_gen import sensors_pb2_grpc, sensors_pb2
from collections import deque
import matplotlib.pyplot as plt
import time

# TODO, decouple plotting from data acquisition for better performance and flexibility

def main():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = sensors_pb2.SensorStreamRequest(queue_size=100)
        stream = stub.getImu(request)
        last_time = 0
     

        # Initialize plot and data buffers outside the loop
        window_size = 100
        ax_data = {'ax': deque(maxlen=window_size),
               'ay': deque(maxlen=window_size),
               'az': deque(maxlen=window_size),
               'gx': deque(maxlen=window_size),
               'gy': deque(maxlen=window_size),
               'gz': deque(maxlen=window_size),
               'timestamp': deque(maxlen=window_size)}

        plt.ion()
        fig, axs = plt.subplots(2, 1, figsize=(10, 6))
        accel_lines = [
            axs[0].plot([], [], label='ax')[0],
            axs[0].plot([], [], label='ay')[0],
            axs[0].plot([], [], label='az')[0]
        ]
        axs[0].set_ylabel('Accel')
        axs[0].legend(loc='upper left')

        gyro_lines = [
            axs[1].plot([], [], label='gx')[0],
            axs[1].plot([], [], label='gy')[0],
            axs[1].plot([], [], label='gz')[0]
        ]
        axs[1].set_ylabel('Gyro')
        axs[1].legend(loc='upper left')

        last_time = 0
        plot_update_interval = 0.02  # seconds
        last_plot_update = time.time()

        for sample in stream:
            # Store data
            ax_data['ax'].append(sample.ax)
            ax_data['ay'].append(sample.ay)
            ax_data['az'].append(sample.az)
            ax_data['gx'].append(sample.gx)
            ax_data['gy'].append(sample.gy)
            ax_data['gz'].append(sample.gz)
            ax_data['timestamp'].append(sample.timestamp)

            # Print only every N samples (optional, for less console spam)
            # if len(ax_data['ax']) % 10 == 0:
            #     print(f"ax={sample.ax}, ay={sample.ay}, az={sample.az}, gx={sample.gx}, gy={sample.gy}, gz={sample.gz}, timestamp={sample.timestamp}")

            # Update plot only every plot_update_interval seconds
            if time.time() - last_plot_update > plot_update_interval:
                accel_lines[0].set_data(ax_data['timestamp'], ax_data['ax'])
                accel_lines[1].set_data(ax_data['timestamp'], ax_data['ay'])
                accel_lines[2].set_data(ax_data['timestamp'], ax_data['az'])
                axs[0].relim()
                axs[0].autoscale_view()

                gyro_lines[0].set_data(ax_data['timestamp'], ax_data['gx'])
                gyro_lines[1].set_data(ax_data['timestamp'], ax_data['gy'])
                gyro_lines[2].set_data(ax_data['timestamp'], ax_data['gz'])
                axs[1].relim()
                axs[1].autoscale_view()

                plt.pause(0.01)
                last_plot_update = time.time()
            

if __name__ == "__main__":
    main()