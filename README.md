# ml-x_ros_driver

This repsitory is supporting ROS topic `\ml_\pointcloud` of ML-X LiDAR.

## Topic Explanation

`\ml_\pointcloud` is sensor_msgs/PointCloud2. Under the topic, the elements of the fields are following:

1. `x`, `y`, `z`
2. `intensity`
3. `ring`: The row number which the point belongs to
4. `offset_time`: Relative time from base timestamp.
	- ML-X LiDAR provides only the timestamp of each row, so the points in the same row have same offset_time.
	- base timestamp is the timestamp of row[0]
	- TODO) Check whether every point of a same row are captured in the same time or sequentially
		- Most LIO algorithms requires each point's timestamp for motion compensation
		
## Use PTP

For using PTP, refer [ML-X SDK User Guide](https://github.com/SOSLAB-github/ML-X_SDK/blob/main/User_Guide/ML-X_User_Guide_v2.3.2(EN).pdf)

In ml-x_ros_driver, there is `start_ptp.sh` which supports start/stop/restart/status of ptp.

### PTP script setting

You shoud change `$INTERFACE` into your connection info.
For checking the connection, command `ifconfig`

After editing the script, do following commands: 
	  
```
$ echo "alias ptp='~/catkin_ws/src/ml/start_ptp.sh'" >> ~/.bashrc
$ source ~/.bashrc
```

### Usage

1. `ptp start`   : start ptp
2. `ptp stop`    : stop ptp
3. `ptp restart` : restart ptp
4. `ptp status`  : show the status of pip
