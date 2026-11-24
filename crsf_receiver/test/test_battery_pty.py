#!/usr/bin/env python3
"""BatteryState -> real ROS node -> CRSF UART, using PTYs only."""
import math
import os
import pty
import signal
import struct
import subprocess
import tempfile
import tty
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from crsf_receiver_msg.msg import CRSFChannels16
from sensor_msgs.msg import BatteryState
from rclpy.qos import qos_profile_sensor_data
from test_serial_pty import channels, frame, spin_for, wait_for


def battery():
    msg = BatteryState()
    msg.present = True
    msg.voltage = 25.2
    msg.current = -12.3
    msg.capacity = 10.0
    msg.charge = 7.5
    msg.percentage = 0.75
    return msg


EXPECTED = frame(0x08, struct.pack(">HH", 252, 123) + (2500).to_bytes(3, "big") + bytes([75]))


class Session:
    def __init__(self, node, namespace, enabled=False, timeout=0.4):
        self.node = node
        self.tmp = tempfile.TemporaryDirectory(prefix="crsf-battery-pty-")
        self.device = Path(self.tmp.name) / "receiver"
        self.master = self.slave = None
        self.data = bytearray()
        self.frames = []
        self.received = []
        self.log = open(Path(self.tmp.name) / "node.log", "w+")
        self.proc = None
        self.sub = node.create_subscription(CRSFChannels16, f"/{namespace}/rc/channels",
                                             self.received.append, qos_profile_sensor_data)
        self.pub = node.create_publisher(BatteryState, f"/{namespace}/battery_input",
                                          qos_profile_sensor_data)
        self.open_pty()
        executable = Path(get_package_prefix("crsf_receiver")) / "lib/crsf_receiver/crsf_receiver_node"
        config = Path(get_package_share_directory("crsf_receiver")) / "config/crsf_receiver.config.yaml"
        command = [
            str(executable), "--ros-args", "--params-file", str(config),
            "-r", f"__ns:=/{namespace}", "-p", f"device:={self.device}",
            "-p", "telemetry_battery_topic:=battery_input", "-p", "telemetry_rate:=5.0",
            "-p", f"telemetry_timeout:={float(timeout)}",
        ]
        if enabled:
            command.extend(["-p", "telemetry_enabled:=true"])
        self.proc = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)

    def __enter__(self):
        try:
            wait_for(self.node, lambda: self.received, action=self.tick)
            return self
        except BaseException:
            self.__exit__(*__import__("sys").exc_info())
            raise

    def open_pty(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.device.symlink_to(os.ttyname(self.slave))

    def read(self):
        while True:
            try:
                data = os.read(self.master, 4096)
            except BlockingIOError:
                break
            if not data:
                break
            self.data.extend(data)
        while len(self.data) >= 2:
            assert self.data[0] == 0xC8 and self.data[1] == 10, bytes(self.data)
            size = self.data[1] + 2
            if len(self.data) < size:
                break
            self.frames.append(bytes(self.data[:size]))
            del self.data[:size]

    def tick(self, msg=None, marker=992):
        os.write(self.master, channels(marker))
        if msg is not None:
            self.pub.publish(msg)
        self.read()

    def reconnect(self):
        os.close(self.master)
        os.close(self.slave)
        self.device.unlink()
        self.master = self.slave = None
        self.data.clear()
        self.frames.clear()
        self.open_pty()
        wait_for(self.node, lambda: self.received[-1].ch1 == 1811,
                 action=lambda: self.tick(marker=1811))

    def __exit__(self, exc_type, exc, tb):
        if self.proc is not None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
                raise
        if exc_type or (self.proc is not None and self.proc.returncode):
            self.log.flush()
            self.log.seek(0)
            print(self.log.read())
        for fd in (self.master, self.slave):
            if fd is not None:
                os.close(fd)
        self.node.destroy_publisher(self.pub)
        self.node.destroy_subscription(self.sub)
        self.log.close()
        self.tmp.cleanup()
        if not exc_type and self.proc is not None:
            assert self.proc.returncode == 0, self.proc.returncode


def main():
    rclpy.init()
    node = rclpy.create_node("battery_pty_test")
    try:
        with Session(node, "battery_disabled") as session:
            spin_for(node, 0.6, lambda: session.tick(battery()))
            assert session.pub.get_subscription_count() == 0
            assert not session.frames and not session.data
            print("PASS: disabled by default: no battery subscription or UART transmission")

        with Session(node, "battery_enabled", enabled=True) as session:
            wait_for(node, lambda: session.pub.get_subscription_count() == 1)
            spin_for(node, 0.3, session.tick)
            assert not session.frames, "sent telemetry before a battery sample"
            initial_rc_count = len(session.received)
            spin_for(node, 1.1, lambda: session.tick(battery()))
            assert 3 <= len(session.frames) <= 7, len(session.frames)
            assert all(packet == EXPECTED for packet in session.frames), session.frames
            assert len(session.received) > initial_rc_count + 5, "RX stalled during TX"
            print("PASS: typed ROS input, configured relative topic, exact CRSF frame/CRC, rate limit, duplex RX")

            spin_for(node, 0.8, session.tick)
            session.frames.clear()
            spin_for(node, 0.4, session.tick)
            assert not session.frames, "replayed stale battery sample"
            invalid = battery()
            invalid.voltage = math.nan
            spin_for(node, 0.4, lambda: session.tick(invalid))
            assert not session.frames, "transmitted invalid battery values"
            wait_for(node, lambda: session.frames, action=lambda: session.tick(battery()))
            session.frames.clear()
            absent = battery()
            absent.present = False
            spin_for(node, 0.3, lambda: session.tick(absent))
            session.frames.clear()
            spin_for(node, 0.4, session.tick)
            assert not session.frames, "replayed previous valid battery after absent battery"
            print("PASS: stale, invalid and absent batteries stop transmission; fresh data resumes it")

        # A long timeout makes the old sample still fresh across reconnection.
        # The node must reset it, rather than replaying it to the new device.
        with Session(node, "battery_reconnect", enabled=True, timeout=10.0) as session:
            wait_for(node, lambda: session.frames, action=lambda: session.tick(battery()))
            session.reconnect()
            spin_for(node, 0.5, session.tick)
            assert not session.frames, "replayed pre-reconnect battery sample"
            wait_for(node, lambda: session.frames, action=lambda: session.tick(battery()))
            assert all(packet == EXPECTED for packet in session.frames)
            print("PASS: UART reconnect resets telemetry and resumes on fresh ROS input; clean shutdown")
        executable = Path(get_package_prefix("crsf_receiver")) / "lib/crsf_receiver/crsf_receiver_node"
        for parameter in ("telemetry_rate", "telemetry_timeout"):
            result = subprocess.run([str(executable), "--ros-args", "-p", "telemetry_enabled:=true",
                                     "-p", parameter + ":=0.0"],
                                    capture_output=True, text=True, timeout=4)
            assert result.returncode == 1 and parameter in result.stderr
        print("PASS: invalid telemetry rate and timeout fail cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
