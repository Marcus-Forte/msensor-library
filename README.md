# The MSensor Library

This library serve as a base for writing generic LiDAR and IMU drivers based on a simple interface.
It was inspired by ROS2, but with gRPC+Protobuf as message and service definition technologies
and simplicity in mind.

By inheriting these common interfaces, the driver benefit from:

* Exposing your driver as an gRPC interface to be used remotely.
* Enabling your driver to serialize data to a file to be read back at a later time.

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




