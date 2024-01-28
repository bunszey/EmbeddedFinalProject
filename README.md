# EmbeddedFinalProject

# Running the project
### Make port read and writable for the USB2AX
```bash
sudo chmod a+rw /dev/ttyACM0
```

### Source ros2
```bash
source /opt/ros/foxy/setup.bash
```

### Build local ros package
```bash
colcon build
```

### Source local ros package
```bash
source install/local_setup.bash
```

### Terminal send ROS message to get position
```bash
ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
```

### Terminal send ROS message to set position
```bash
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 50}"
```

### Run camera node
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p image_width:=320 -p image_height:=240
```

### Terminal send ROS message to set position
```bash
ros2 topic pub -1 /gotopos std_msgs/Int32 "{data: 1}"
```


## Terminal launch project and send ROS message to start system
```bash
ros2 launch project_launch project_launch.py 
```
```bash
ros2 topic pub -1 /start std_msgs/String "{data: 0}"
```

