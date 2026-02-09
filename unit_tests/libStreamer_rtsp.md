# rtsp Unit Testing
The following instructions have been developed for regression testing of the rtsp module.

## Pre-requisites
The tests below are expected to be relative to your home directory and as such use `~/rosVid` as the expected directory from where to call.

## Building
```bash
cd ~/rosVid/ros2_ws
colcon build
```

## ROS2 test
Open a shell and do:
```bash
python3 ~/rosVid/rtspServer.py
``` 

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures rtsp
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures watcher --ros-args -p capture:=rtsp
```

With the shell commands having been ran a watcher GUI should appear showing you what the video camera sees.  If this test passes, proceed by closing out the open shells.

## Python test
With the phys module installed in your Python environment, open a shell and do:
```bash
python3 ~/rosVid/rtspServer.py
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
python3 << 'EOF'
import rclpy
import threading
from libStreamer_rtsp import Rtsp
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

rclpy.init()
node = Node('rtsp')
rts = Rtsp(node, args = {'pubSpeed': 15})

executor = MultiThreadedExecutor()
executor.add_node(node)
theThread = threading.Thread(daemon = True, target = executor.spin)
theThread.start()
theThread.join()
EOF
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures watcher --ros-args -p pickled:=True -p capture:=rtsp
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 topic pub /captures/rtsp/config std_msgs/String "data: '{height: 720, width: 1280}'" --once
sleep 1
ros2 topic pub /captures/rtsp/config std_msgs/String "data: 'pubSpeed: 20'" --once
sleep 1
ros2 topic pub /captures/rtsp/config std_msgs/String "data: '{height: 480, width: 640}'" --once
sleep 1
ros2 topic pub /captures/rtsp/config std_msgs/String "data: 'pubSpeed: 1'" --once
```

With the above shell commands having been ran a watcher GUI should appear and have been manipulated accordingly.  If this test passes, proceed by closing out the shells.