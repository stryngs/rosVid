# udp Unit Testing
The following instructions have been developed for regression testing of the udp module.

## Pre-requisites
The tests below are expected to be relative to your home directory and as such use `~/rosVid` as the expected directory from where to call.

## Building
```bash
cd ~/rosVid/ros2_ws
colcon build
```

## UDP Stream
Any kind of UDP streaming will do just fine.  For the purposes of a repeatable test, Gazebo using [this](https://github.com/ArduPilot/ardupilot_gazebo) is the workflow:
```bash
gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
source "/home/$USER/rosVid/ros2_ws/install/setup.bash"
ros2 run captures udp
```

## ROS2 test
Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures udp
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures watcher
```

With both shell commands having been ran a watcher GUI should appear showing you what the video camera sees.  If this test passes, proceed by closing out the open shells.

## Python test
With the udp module installed in your Python environment, open a shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
python3 << 'EOF'
import rclpy
import threading
from libStreamer_udp import Udp
from rclpy.node import Node

rclpy.init()
node = Node('udp')
udp = Udp(node, args = {'pubSpeed': 15})

theThread = threading.Thread(daemon = True, target = lambda: rclpy.spin(node))
theThread.start()
theThread.join()
EOF
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures watcher --ros-args -p capture:=udp -p annotated:=True
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 topic pub /captures/udp/config std_msgs/String "data: '{height: 720, width: 1280}'" --once
sleep 1
ros2 topic pub /captures/udp/config std_msgs/String "data: 'pubSpeed: 20'" --once
sleep 1
ros2 topic pub /captures/udp/config std_msgs/String "data: '{height: 480, width: 640}'" --once
sleep 1
ros2 topic pub /captures/udp/config std_msgs/String "data: 'pubSpeed: 1'" --once
```

With the above shell commands having been ran a watcher GUI should appear and have been manipulated accordingly.  If this test passes, proceed by closing out the shells.