## Description


**ROS 2** package for receiving CRSF (RC channels values) packets over serial port (UART).


### Topics
    
- `rc/channels` - received rc channels values
- `rc/link` - connection statistics information


---

## Installation



### Dependencies:

Install `CppLinuxSerial` before:

```bash
git clone https://github.com/gbmhunter/CppLinuxSerial.git

cd CppLinuxSerial
mkdir build/
cd build
cmake ..
make
sudo make install
cd ../..
```


Let's assume that your ros 2 workspace localized at `~/row2_ws/`.


### 1. Clone package from git:

```bash
cd ~/row2_ws/src

# Types dependency package:
git clone https://github.com/AndreyTulyakov/ros2_crsf_receiver.git
```

### 2. Build

```bash
cd ~/row2_ws

colcon build --packages-select crsf_receiver_msg
colcon build --packages-select crsf_receiver
```

### 3. Re-source

```bash
source ~/row2_ws/install/setup.bash
```

---



## Running


### Set up params:

1. Serial device name: `device`, default is `/dev/ttyUSB0`
2. Baud rate: `baud_rate`, default is `420000`
3. Enable / Disable link statistics info: `link_stats`, default is `false`
4. Receiver rate (hz): `receiver_rate`, default is `100`


### Run ros node:

```bash
# Run Node with default parameters
ros2 run crsf_receiver crsf_receiver_node

# Or setup and run Node with custom parameters values:
ros2 run crsf_receiver crsf_receiver_node --ros-args -p "device:=/dev/serial0" -p baud_rate:=420000  -p link_stats:=true
```

### Check

After correct setup and running without errors you can check topics:

```bash
# Check channels values
ros2 topic echo /rc/channels

# Check link statisics
ros2 topic echo /rc/link

# Check receiver rate
ros2 topic hz /rc/channels
```



###  Link statistics message fields:

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
