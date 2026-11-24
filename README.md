# CRSF receiver for ROS 2

Receive RC channels and link statistics from an ExpressLRS or TBS Crossfire
receiver over a Linux UART or USB-to-UART adapter. The node uses the
[`serial_driver` and `io_context` libraries](https://github.com/ros-drivers/transport_drivers)
from `ros-drivers/transport_drivers`, with standalone ASIO. CppLinuxSerial is no
longer required. The `serial_bridge` executable is not needed: this node opens
the UART directly through the library.

Protocol reference: [Team BlackSheep CRSF specification](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md).

## Topics

| Relative topic | Message type | Contents |
| --- | --- | --- |
| `rc/channels` | `crsf_receiver_msg/msg/CRSFChannels16` | Sixteen raw 11-bit channel values |
| `rc/link` | `crsf_receiver_msg/msg/CRSFLinkInfo` | Received CRSF link statistics |

Both publishers use best-effort reliability, volatile durability and depth 1.
With no ROS namespace, the topics are `/rc/channels` and `/rc/link`.

## Installation

Requirements: Linux, a configured ROS 2 development installation, colcon,
rosdep, CMake 3.20 or newer and a C++20 compiler. Source your ROS installation
before running the commands below (for example, `source /opt/ros/jazzy/setup.bash`
for Jazzy). Use one ROS distribution per build environment.

### 1. Clone the receiver

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AndreyTulyakov/ros2_crsf_receiver.git
```

### 2. Provide transport_drivers

Choose one of these methods.

**Binary packages**, when available for your ROS distribution:

```bash
sudo apt install "ros-${ROS_DISTRO}-serial-driver" "ros-${ROS_DISTRO}-io-context"
```

**Source build**, if binary packages are unavailable or you want to build the
transport libraries yourself:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ros-drivers/transport_drivers.git
```

For **ROS 2 Lyrical**, upstream revision
`0af598879560099b1069f9ad03d99f10684279ea` still calls the removed
`ament_target_dependencies` CMake function. For a reproducible source build,
apply the included CMake compatibility patch to that revision:

```bash
cd ~/ros2_ws/src
git -C transport_drivers checkout 0af598879560099b1069f9ad03d99f10684279ea
git -C transport_drivers apply --check ../ros2_crsf_receiver/patches/transport_drivers-lyrical.patch
git -C transport_drivers apply ../ros2_crsf_receiver/patches/transport_drivers-lyrical.patch
```

This patch changes only transport_drivers build files. The receiver uses its
public API and requires no transport driver source/header extensions. Do not
apply the Lyrical patch to binary installations or other revisions without
checking whether it is needed and applicable.

### 3. Resolve dependencies and build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --symlink-install --packages-up-to crsf_receiver \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

`--packages-up-to` includes all dependencies present in the workspace, including
`asio_cmake_module`, `io_context` and `crsf_receiver_msg`. Dependencies already
installed with ROS are taken from the sourced ROS installation. The unrelated
`udp_driver` package does not need to be built.

Give your Linux user access to the selected UART device. On Debian/Ubuntu this
usually means membership in `dialout`; log out and back in after changing groups.
Only one application should open the receiver's serial port at a time.

**Verified configuration:** ROS 2 Lyrical, a clean source build of
transport_drivers `0af5988` with the CMake patch above, followed by the C++ tests
and the actual ROS node tested over PTYs. Other ROS distributions are not part
of this validation.

## Running

### Set up params:

1. Serial device name: `device`, default is `/dev/serial0`
2. Baud rate: `baudrate`, default is `420000`
3. Enable / Disable link statistics publication: `link_stats`, default is `true`
4. Maximum publication rate (Hz): `receiver_rate`, default is `100`, allowed range `1..1000`

UART reception runs asynchronously on one I/O thread. The publication timer
selects the newest received sample; it does not publish a sample a second time.
For example, 500 Hz input with `receiver_rate:=100` publishes at most 100 Hz.
No channels or link statistics are published before the first valid frame.
Channels expire after 50 ms and statistics after 1000 ms. Only valid channel
frames keep the parser's RC link state alive (200 ms timeout).
When RC data stops, the channels topic becomes silent; consumers must implement
their own receive timeout rather than holding the last command indefinitely.

Malformed lengths and CRC errors resynchronize one byte at a time without
discarding a following valid frame. Incomplete frames expire after 100 ms without
new bytes. A missing or disconnected serial port is retried once per second.
`device` should be a stable path if USB adapters can change their device number.

This is a receiver-only node: it decodes `0x16` (sixteen raw 11-bit channels,
`0..2047`, center `992`) and `0x14` (link statistics), with `0xC8` framing.
It does not implement every CRSF frame type or send telemetry to the handset.

**Interface change:** `CRSFLinkInfo.uplink_snr` is now `std_msgs/Int8`, matching
the CRSF signed SNR field and `downlink_snr`. Rebuild message consumers and restart
all publishers/subscribers together; do not mix old and new interface builds.

### Run ros node:

```bash
# Run Node with default parameters
ros2 run crsf_receiver crsf_receiver_node

# Load the installed default configuration
ros2 run crsf_receiver crsf_receiver_node --ros-args \
  --params-file "$(ros2 pkg prefix --share crsf_receiver)/config/crsf_receiver.config.yaml"

# Or setup and run Node with custom parameters values:
ros2 run crsf_receiver crsf_receiver_node --ros-args -p "device:=/dev/ttyUSB0" -p baudrate:=420000 -p link_stats:=true
```

The default configuration is stored in
[`crsf_receiver/config/crsf_receiver.config.yaml`](crsf_receiver/config/crsf_receiver.config.yaml)
and installed under `share/crsf_receiver/config`. Its `/**` selector supports
renamed and namespaced nodes. To customize it, copy the file and pass the copy
with `--params-file`; command-line `-p` arguments placed afterward can override
individual values, for example `-p device:=/dev/ttyUSB0`.

### Check

After correct setup and running without errors you can check topics:

```bash
# Check channels values
ros2 topic echo /rc/channels --qos-reliability best_effort

# Check link statistics
ros2 topic echo /rc/link --qos-reliability best_effort

# Check receiver rate
ros2 topic hz /rc/channels
```

### Link statistics message fields:

- `uplink_rssi_ant1` - ( dBm * -1 )
- `uplink_rssi_ant2` - ( dBm * -1 )
- `uplink_status` - Package success rate / Link quality ( % )
- `uplink_snr` - ( db )
- `active_antenna` - Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
- `rf_mode` - ( enum 4fps = 0 , 50fps, 150hz)
- `uplink_tx_power` - ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
- `downlink_rssi` - ( dBm * -1 )
- `downlink_status` - Downlink package success rate / Link quality ( % )
- `downlink_snr` - ( db )

## Tests

After the initial build above, run these commands from the ROS workspace root.
Only the receiver needs to be reconfigured with tests enabled; transport_drivers
can keep `BUILD_TESTING=OFF`.

```bash
source install/setup.bash
colcon build --symlink-install --packages-select crsf_receiver --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select crsf_receiver
colcon test-result --verbose
# Use an unused ROS domain; this opens only temporary PTYs, no robot devices:
ROS_DOMAIN_ID=151 python3 src/ros2_crsf_receiver/crsf_receiver/test/test_serial_pty.py
```

The C++ tests cover startup validity, all channel bits, every frame split point,
CRC and length errors, the repeated-sync regression from the Godbolt comment,
partial-frame recovery, signed SNR, freshness and randomized input. The PTY test
runs the actual ROS node, including device appearance, disconnect/reconnect,
`link_stats:=false`, sample deduplication and shutdown during serial input.
