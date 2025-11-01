# The MSensor Library

This library serve as an experiment for writing generic LiDAR and IMU drivers based on a simple interface.
It was inspired by ROS2, but with gRPC+Protobuf as message and service definition technologies
and simplicity in mind.

By inheriting these common interfaces, the driver benefit from:

* Exposing your driver as an gRPC interface to be used remotely.
* Enabling your driver to serialize data to a file to be read back at a later time.

## Development

Two devcontainers are included. The `dev` is a normal devcontainer for development.
The `envoy` is a special devcontainer that pulls envoy proxy that proxies grpc-web HTTP request to gRPC of the container.

## Usage

### As an interface

* The interface at `include/lidar/ILidar` serves as a base for implementing new LiDAR drivers.
* The interface at `include/imu/IImu` serves as a base for implementing new IMU drivers.

### Remote Driver

* Your driver can be programmed as a gRPC service. See `src/*_publisher` applications as example.
* A client (at another machine) application can instantiate a `grpc/sensors_remote_client.hh` class and subscribe to the IP
of a driver server to get sensor data.

### Client

A Python client is provided with the repository to facilitate testing the remote driver.
See `test/client` folder.

## Docker

A `DockerfileRuntime` is provided to offer small footprint images that allows one to run the publisher applications from inside a container. Make sure the hardware is correctly mapped to the container (`--device /dev/ttyUSB*` or `--network=host`)

Supported docker build targets are:
* `rplidar`
* `mid360`
* `icm20948`

Example with Mid360 LIDAR+IMU:
* build: `docker build -f docker/DockerfileRuntime -t mid360 --target icm20948 .`
* run: `docker run --network host --rm mid360 <nr samples>`

Example ICM20948 IMU:
* build: `docker build -f docker/DockerfileRuntime -t icm20948 --target icm20948 .`
* run: `docker run --rm --init --device /dev/i2c-1 -p 50051:50051 icm20948`

