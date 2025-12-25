import argparse

import grpc
from proto_gen import sensors_pb2_grpc, sensors_pb2

import viser
import viser.uplot

import numpy as np

DEFAULT_SERVER_ADDR = "localhost:50051"


def main():
    parser = argparse.ArgumentParser(description="Stream IMU data over gRPC.")
    parser.add_argument("--server", default=DEFAULT_SERVER_ADDR, help="gRPC target host:port")
    args = parser.parse_args()

    server = viser.ViserServer()

    time_step = 1.0 / 60.0
    num_timesteps = 100
    num_signals = 3  # ax, ay, az

    x_data = time_step * np.arange(num_timesteps, dtype=np.float64)
    y_acc = np.zeros((num_signals, num_timesteps))
    y_gyr = np.zeros((num_signals, num_timesteps))
    data_gyro = (x_data, *y_gyr)
    data_acc = (x_data, *y_acc)

    print("Data shapes:", [arr.shape for arr in data_acc])

    uplot_handles: list[viser.GuiUplotHandle] = []
    uplot_handles.append(
        server.gui.add_uplot(
            data=data_acc,
            series=(
                viser.uplot.Series(label="time"),
                *[
                    viser.uplot.Series(
                        label=f"y{i}",
                        stroke=["red", "green", "blue"][i % 3],
                        width=2,
                    )
                    for i in range(num_signals)
                ],
            ),
            title="Accelerometer",
            scales={
                "x": viser.uplot.Scale(
                    time=False,
                    auto=True,
                ),
                "y": viser.uplot.Scale(auto=True),
            },
            legend=viser.uplot.Legend(show=True),
            aspect=2.0,
        )
    )

    uplot_handles.append(
        server.gui.add_uplot(
            data=data_gyro,
            series=(
                viser.uplot.Series(label="time"),
                *[
                    viser.uplot.Series(
                        label=f"y{i}",
                        stroke=["red", "green", "blue"][i % 3],
                        width=2,
                    )
                    for i in range(num_signals)
                ],
            ),
            title="Gyro",
            scales={
                "x": viser.uplot.Scale(
                    time=False,
                    auto=True,
                ),
                "y": viser.uplot.Scale(auto=True),
            },
            legend=viser.uplot.Legend(show=True),
            aspect=2.0,
        )
    )

    with grpc.insecure_channel(args.server) as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)
        request = sensors_pb2.SensorStreamRequest(queue_size=1)
        stream = stub.getImu(request)
        lastT = 0
        sample: int = 0
        for imu in stream:
            deltaT = imu.timestamp - lastT
            print(f" DeltaT: {deltaT} ms")
            lastT = imu.timestamp
            sample += 1
            # Update the line plot.
            y_acc[:, sample % num_timesteps] = (imu.ax, imu.ay, imu.az)

            # Update Acc
            uplot_handles[0].data = (x_data, *y_acc)

            y_gyr[:, sample % num_timesteps] = (imu.gx, imu.gy, imu.gz)

            # Update Gyro
            uplot_handles[1].data = (x_data, *y_gyr)


if __name__ == "__main__":
    main()
