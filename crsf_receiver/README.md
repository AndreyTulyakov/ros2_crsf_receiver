# crsf_receiver

ROS 2 node for receiving CRSF RC channels and link statistics through the
`serial_driver` and `io_context` libraries from `ros-drivers/transport_drivers`.
Optional battery telemetry subscribes to `sensor_msgs/msg/BatteryState` and
sends CRSF `0x08` frames back to the receiver; it is disabled by default.

See the [repository README](../README.md) for dependencies, installation,
parameters, supported frames and tests.
