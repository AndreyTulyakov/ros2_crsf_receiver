#!/usr/bin/env python3
"""Exercise the actual ROS node over PTYs; no robot hardware is opened.

Run after sourcing the freshly built workspace, in an unused ROS domain:
ROS_DOMAIN_ID=151 python3 crsf_receiver/test/test_serial_pty.py
"""
import os
import pty
import signal
import subprocess
import tempfile
import time
import tty
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from crsf_receiver_msg.msg import CRSFChannels16, CRSFLinkInfo
from rclpy.qos import qos_profile_sensor_data


def frame(kind, payload):
    body = bytes([kind]) + payload
    crc = 0
    for byte in body:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 255
    return bytes([0xC8, len(body) + 1]) + body + bytes([crc])


def channels(marker):
    values = [marker] + [992] * 15
    packed = sum(value << (11 * i) for i, value in enumerate(values))
    return frame(0x16, packed.to_bytes(22, "little"))


def spin_for(node, duration, action=None):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if action:
            action()
        rclpy.spin_once(node, timeout_sec=0.02)


def wait_for(node, predicate, timeout=4, action=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if action:
            action()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError("condition timed out")


def run_case(node, binary, link_stats):
    namespace = "crsf_pty_stats" if link_stats else "crsf_pty_no_stats"
    received, links = [], []
    subs = [
        node.create_subscription(CRSFChannels16, f"/{namespace}/rc/channels", received.append,
                                 qos_profile_sensor_data),
        node.create_subscription(CRSFLinkInfo, f"/{namespace}/rc/link", links.append,
                                 qos_profile_sensor_data),
    ]
    master = slave = None
    with tempfile.TemporaryDirectory(prefix="crsf-pty-") as tmp:
        device = Path(tmp) / "receiver"
        with open(Path(tmp) / "node.log", "w+") as log:
            proc = subprocess.Popen([
                str(binary), "--ros-args", "-r", f"__ns:=/{namespace}",
                "-p", f"device:={device}", "-p", "baudrate:=420000",
                "-p", f"link_stats:={str(link_stats).lower()}",
            ], stdout=log, stderr=subprocess.STDOUT)
            try:
                wait_for(node, lambda: all(node.count_publishers(s.topic_name) for s in subs))
                spin_for(node, 0.25)
                assert not received and not links, "published data before any UART input"
                # Start without a device: the node must reconnect when it appears.
                master, slave = pty.openpty()
                tty.setraw(slave)
                device.symlink_to(os.ttyname(slave))
                wait_for(node, lambda: received and received[-1].ch1 == 172,
                         action=lambda: os.write(master, channels(172)))
                assert all(m.ch2 == 992 and m.ch16 == 992 for m in received)

                spin_for(node, 0.1)
                link = frame(0x14, bytes([70, 75, 99, 249, 1, 2, 3, 80, 95, 252]))
                os.write(master, link)
                if link_stats:
                    wait_for(node, lambda: len(links) == 1)
                    assert links[0].uplink_snr.data == -7
                    assert links[0].downlink_snr.data == -4
                spin_for(node, 0.15)
                count = len(received)
                assert len(links) == int(link_stats), "duplicated or disabled statistics published"
                spin_for(node, 0.15, lambda: os.write(master, frame(0x16, b"\0")))
                assert len(received) == count, "malformed channels renewed control"
                spin_for(node, 0.15)
                assert len(received) == count, "republished an old channel sample"

                # HUP, a missing path, then a different UART at the same stable path.
                os.close(master)
                os.close(slave)
                master = slave = None
                device.unlink()
                spin_for(node, 0.25)
                master, slave = pty.openpty()
                tty.setraw(slave)
                device.symlink_to(os.ttyname(slave))
                wait_for(node, lambda: received and received[-1].ch1 == 1811,
                         action=lambda: os.write(master, channels(1811)))
                assert proc.poll() is None
                # Stop under active input to exercise callback destruction order.
                os.write(master, channels(992) * 100)
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=4)
                assert proc.returncode == 0, proc.returncode
                print(f"PASS: PTY link_stats={link_stats}: silent startup, decode, SNR, "
                      "no repeats, malformed input, reconnect, shutdown")
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
                for fd in (master, slave):
                    if fd is not None:
                        os.close(fd)
    for sub in subs:
        node.destroy_subscription(sub)


def main():
    rclpy.init()
    node = rclpy.create_node("crsf_pty_test")
    binary = Path(get_package_prefix("crsf_receiver")) / "lib/crsf_receiver/crsf_receiver_node"
    try:
        run_case(node, binary, True)
        run_case(node, binary, False)
        proc = subprocess.run([str(binary), "--ros-args", "-p", "receiver_rate:=0"],
                              capture_output=True, text=True, timeout=4)
        assert proc.returncode == 1 and "receiver_rate" in proc.stderr
        print("PASS: invalid parameters fail cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
