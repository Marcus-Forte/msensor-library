import argparse
import threading
import time
from typing import Sequence

import grpc
import numpy as np
import viser
import viser.uplot

from proto_gen import sensors_pb2, sensors_pb2_grpc


DEFAULT_SERVER_ADDR = "localhost:50051"
DEFAULT_ADC_CHANNEL = 0
DEFAULT_ADC_PERIOD_SEC = 0.5


def button_callback(event):
    print(f"Button clicked! Event: {event}")


def to_numpy(points: Sequence[sensors_pb2.Point3]) -> np.ndarray:
    arr = np.zeros((len(points), 3), dtype=np.float32)
    for i, point in enumerate(points):
        arr[i, 0] = point.x
        arr[i, 1] = point.y
        arr[i, 2] = point.z
    return arr


def setup_imu_plots(server: viser.ViserServer):
    time_step = 1.0 / 60.0
    num_timesteps = 1000
    num_signals = 3

    x_data = time_step * np.arange(num_timesteps, dtype=np.float64)
    y_acc = np.zeros((num_signals, num_timesteps))
    y_gyr = np.zeros((num_signals, num_timesteps))
    data_acc = (x_data, *y_acc)
    data_gyro = (x_data, *y_gyr)

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
                "x": viser.uplot.Scale(time=False, auto=True),
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
                "x": viser.uplot.Scale(time=False, auto=True),
                "y": viser.uplot.Scale(auto=True),
            },
            legend=viser.uplot.Legend(show=True),
            aspect=2.0,
        )
    )

    button = server.gui.add_button("button")
    button.on_click(button_callback)

    return {
        "x_data": x_data,
        "y_acc": y_acc,
        "y_gyr": y_gyr,
        "num_timesteps": num_timesteps,
        "uplot_handles": uplot_handles,
    }


def stream_imu(stub: sensors_pb2_grpc.SensorServiceStub, server: viser.ViserServer, stop_event: threading.Event):
    context = setup_imu_plots(server)
    request = sensors_pb2.SensorStreamRequest(queue_size=1)
    sample = 0

    try:
        for imu in stub.getImu(request):
            if stop_event.is_set():
                break
            sample += 1
            context["y_acc"][:, sample % context["num_timesteps"]] = (imu.ax, imu.ay, imu.az)
            context["uplot_handles"][0].data = (context["x_data"], *context["y_acc"])

            context["y_gyr"][:, sample % context["num_timesteps"]] = (imu.gx, imu.gy, imu.gz)
            context["uplot_handles"][1].data = (context["x_data"], *context["y_gyr"])
    except grpc.RpcError as exc:
        print(f"IMU stream error: {exc.code().name} - {exc.details()}")


def stream_lidar(stub: sensors_pb2_grpc.SensorServiceStub, server: viser.ViserServer, stop_event: threading.Event):
    request = sensors_pb2.SensorStreamRequest(queue_size=100)
    last_timestamp = 0

    try:
        for scan in stub.getScan(request):
            if stop_event.is_set():
                break
            print(f"Got points: {len(scan.points)} at {scan.timestamp}")
            delta_t = scan.timestamp - last_timestamp
            print(f" DeltaT: {delta_t} ms")
            last_timestamp = scan.timestamp

            server.scene.add_point_cloud(
                name="/lidar",
                points=to_numpy(scan.points),
                colors=(255, 0, 0),
                point_size=0.05,
                point_shape="rounded",
            )
    except grpc.RpcError as exc:
        print(f"LiDAR stream error: {exc.code().name} - {exc.details()}")


def stream_adc(
    stub: sensors_pb2_grpc.SensorServiceStub,
    stop_event: threading.Event,
    channel: int,
    period_sec: float,
):
    request = sensors_pb2.AdcDataRequest(channel=channel)

    print(f"Polling ADC channel {channel} every {period_sec}s. Press Ctrl+C to stop.")
    while not stop_event.is_set():
        try:
            resp = stub.GetAdc(request)
            print(f"ADC{channel}: {resp.sample:.4f} V @ {resp.timestamp}")
        except grpc.RpcError as exc:
            print(f"ADC error: {exc.code().name} - {exc.details()}")
        stop_event.wait(period_sec)


def parse_args():
    parser = argparse.ArgumentParser(description="Subscribe to multiple sensor streams over gRPC.")
    parser.add_argument("--server", default=DEFAULT_SERVER_ADDR, help="gRPC target host:port")
    parser.add_argument("--imu", action="store_true", help="Subscribe to the IMU stream")
    parser.add_argument("--lidar", action="store_true", help="Subscribe to the LiDAR stream")
    parser.add_argument("--adc", action="store_true", help="Poll the ADC")
    parser.add_argument("--adc-channel", type=int, default=DEFAULT_ADC_CHANNEL, help="ADC channel to poll")
    parser.add_argument(
        "--adc-period",
        type=float,
        default=DEFAULT_ADC_PERIOD_SEC,
        help="Seconds between ADC polls",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if not any([args.imu, args.lidar, args.adc]):
        args.imu = args.lidar = args.adc = True

    needs_visualization = args.imu or args.lidar
    server = viser.ViserServer() if needs_visualization else None

    stop_event = threading.Event()
    threads: list[threading.Thread] = []

    with grpc.insecure_channel(args.server) as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)

        if args.imu:
            if server is None:
                raise RuntimeError("IMU visualization requires viser to be available.")
            t = threading.Thread(target=stream_imu, args=(stub, server, stop_event), name="imu-thread")
            t.start()
            threads.append(t)

        if args.lidar:
            if server is None:
                raise RuntimeError("LiDAR visualization requires viser to be available.")
            t = threading.Thread(target=stream_lidar, args=(stub, server, stop_event), name="lidar-thread")
            t.start()
            threads.append(t)

        if args.adc:
            t = threading.Thread(
                target=stream_adc,
                args=(stub, stop_event, args.adc_channel, args.adc_period),
                name="adc-thread",
            )
            t.start()
            threads.append(t)

        try:
            while any(t.is_alive() for t in threads):
                for t in threads:
                    t.join(timeout=0.5)
        except KeyboardInterrupt:
            print("\nStopping sensor subscriptions...")
            stop_event.set()

    for t in threads:
        t.join()


if __name__ == "__main__":
    main()
