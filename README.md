# ML-X ROS Driver

This repository provides a ROS driver for ML-X LiDAR sensors, supporting both single and multi-LiDAR configurations with synchronized scanning capabilities.

## Features

- Publish point cloud data via ROS topic: `/ml_/pointcloud`
- Support for multiple ML-X LiDARs with synchronized scanning
- Precision Time Protocol (PTP) synchronization support
- Configuration of sensor modes (ambient, depth, intensity, multi-echo)

## Topic Information

The `/ml_/pointcloud` topic publishes sensor_msgs/PointCloud2 data with the following fields:

1. `x`, `y`, `z` - 3D coordinates
2. `intensity` - Intensity values 
3. `ring` - Row number that the point belongs to
4. `offset_time` - Relative time from base timestamp
   - ML-X LiDAR provides the timestamp for each row, so points in the same row have the same offset_time
   - Base timestamp is the timestamp of row[0]

## Usage

### Single LiDAR

```
roslaunch ml ml.launch
```

### Multiple LiDARs with Synchronization

```
roslaunch ml ml_multi.launch
```

**Note:** The current version supports up to two LiDARs in multi mode. Support for more than two LiDARs will be added in a future update.

#### Important: IP Configuration for Multi-LiDAR Setup

When using multiple ML-X LiDARs, each device must have a unique IP address configuration:

1. **Device IP Configuration**: Each LiDAR's IP address must be configured both in:
   - The launch file (`ml_multi.launch`)
   - The LiDAR device itself using SOS Studio (refer to [SOS Studio Usage (p.22) of User Guide](https://github.com/SOSLAB-github/ML-X_SDK/blob/main/User_Guide/ML-X_User_Guide_v2.3.2(EN).pdf))

2. **Different Subnet Requirements**: Each LiDAR must be on a different subnet (third octet of IPv4 address) to prevent network conflicts.

   Example configuration:
   - LiDAR 1: Device 192.168.1.10, PC port 192.168.1.11
   - LiDAR 2: Device 192.168.2.10, PC port 192.168.2.11

   Note that the subnet (third number) is different (1 vs 2) for each LiDAR. Using the same subnet for multiple LiDARs can cause communication conflicts and data loss.

#### Launch File Parameters

The multi-LiDAR launch file includes synchronization parameters:
- `use_sync_start`: Enable synchronized start of scanning
- `expected_nodes`: Number of LiDARs to synchronize
- `sync_timeout`: Maximum wait time for synchronization
- `sync_node_list`: List of LiDAR node names to synchronize

## Time Synchronization

The `sync.sh` script provides PTP (Precision Time Protocol) and PHC2SYS synchronization for precise timing between multiple LiDARs and the system clock.

### Configuration

Edit the `INTERFACES` array in `sync.sh` to match your network interfaces:
```
INTERFACES=("eth0" "eth1")  # adjust as needed
```

#### Checking Network Interfaces

1. Identify your network interfaces:
   ```
   ifconfig
   ```
   This will list all available network interfaces (eth0, eth1, enp0s31f6, etc.) on your system.

2. Verify hardware-based PTP support:
   ```
   ethtool -T eth0
   ```
   Look for any of these indicators of hardware timestamp support:
   - "hardware-timestamps" in capabilities
   - "HARDWARE_TRANSMIT" or "hardware-transmit" 
   - "HARDWARE_RECEIVE" or "hardware-receive"
   - "SOF_TIMESTAMPING_TX_HARDWARE"
   - "SOF_TIMESTAMPING_RX_HARDWARE"
   
   If these capabilities are present, your network interface supports hardware-based PTP, which provides more accurate timing than software-based solutions.

For easier usage, create an alias:
```
echo "alias ptp='~/catkin_ws/src/ml-x_ros_driver/sync.sh'" >> ~/.bashrc
source ~/.bashrc
```

### Commands

- `ptp start` - Start PTP synchronization
- `ptp stop` - Stop PTP services
- `ptp restart` - Restart PTP services
- `ptp status` - Check PTP status
- `ptp monitor {1|2}` - Monitor PTP with different verbosity levels

For more information on PTP, refer to the [ML-X SDK User Guide](https://github.com/SOSLAB-github/ML-X_SDK/blob/main/User_Guide/ML-X_User_Guide_v2.3.2(EN).pdf)