# The MSensor Library

This library serve as a base for writing generic LiDAR and IMU drivers based on a simple interface.
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
* The interface at `include/lidar/IImu` serves as a base for implementing new IMU drivers.

### As a driver

* The classes at `src/lidar` represent LiDARs objects / drivers that can be instantiated and communicate with the real LiDAR sensors.
* The classes at `src/imu` represent IMU objects / drivers that can be instantiated and communicate with the real IMU sensors.

### Remote Driver

* Your driver can be programmed as a gRPC service. See `src/*_publisher` applications as example.
* A client (at another machine) application can instantiate a `grpc/sensors_client.hh` class and subscribe to the IP
of a driver server to get sensory data.

### Client

A Python client is provided together with the repository to facilitate testing the remote driver.
See `test/client` folder.

## Docker

A `DockerfileRuntime` is provided to offer small footprint images that allows one to run the publisher applications from inside a container. Make sure the hardware is correctly mapped to the container (`--device /dev/ttyUSB*` or `--network=host`)

To build the Mid360 publisher for example, use:
`docker build -f docker/DockerfileRuntime -t mid360 .`

To run, use:
`docker run --network host --rm mid360 <nr samples>`

Using ICM20948
`docker run --rm --init --device /dev/i2c-1 -p 50051:50051 imu`

