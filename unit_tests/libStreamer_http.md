# http Unit Testing
The following instructions have been developed for regression testing of the http module.

## Pre-requisites
The tests below are expected to be relative to your home directory and as such use `~/rosVid` and [~/objCounter](https://github.com/stryngs/objCounter) as the expected directories from where to call.

The http module needs an HTTP source.  The repeatable workflow below uses `objCounter`'s `webCounter` endpoint as that source.  `webCounter` serves `/frame.jpg` on demand, even when the browser page itself is configured to use MJPEG.

## Building
```bash
cd ~/rosVid/ros2_ws
colcon build
cd ~/objCounter/ros2_ws
colcon build
```

## HTTP Source
Open a shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures phys
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/objCounter
source ros2_ws/install/setup.bash
ros2 run obj_counter objCounter --ros-args \
 -p model:=./detections/ssd_mobilenet_v2_coco_quant_postprocess.tflite \
 -p labelsFile:=./detections/labels.txt \
 -p capture:=phys \
 -p maxFps:=10 \
 -p dbPath:=./door_counter.sqlite3 \
 -p triggerMode:=center \
 -p showCountsOverlay:=False
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/objCounter/ros2_ws/install/setup.bash
ros2 run obj_counter webCounter --ros-args -p useMjpeg:=True
```

At this point `webCounter` should be listening on port 8002.  The HTTP module can use either of these endpoints:
```text
http://127.0.0.1:8002/frame.jpg
http://127.0.0.1:8002/video.mjpg
```

## ROS2 snapshot test
Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures http --ros-args \
  -p httpUrl:=http://127.0.0.1:8002/frame.jpg \
  -p httpMode:=snapshot \
  -p topicId:=_http_test
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 run captures watcher --ros-args \
  -p capture:=http \
  -p compressed:=True \
  -p topicId:=_http_test
```

With the shell commands having been ran a watcher GUI should appear showing the HTTP-ingested video.  If this test passes, leave the shells open and continue with the MJPEG test.

## ROS2 MJPEG test
Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 topic pub /captures/http/config_http_test std_msgs/String "data: '{httpUrl: \"http://127.0.0.1:8002/videoX.mjpg\", httpMode: mjpeg}'" --once
```

The watcher GUI should now be paused, but still open.

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 topic pub /captures/http/config_http_test std_msgs/String "data: '{httpUrl: \"http://127.0.0.1:8002/video.mjpg\", httpMode: mjpeg}'" --once
```

The watcher GUI should continue rendering after the HTTP capture reconnects to the MJPEG endpoint.

## Python test
With the http module installed in your Python environment, open a shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
python3 << 'EOF'
import rclpy
import threading
from captures.http import Http
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

rclpy.init()
node = Node('http')
http = Http(node, args = {'httpUrl': 'http://127.0.0.1:8002/frame.jpg',
                          'httpMode': 'snapshot',
                          'pubSpeed': 15,
                          'topicId': '_http_py'})

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
ros2 run captures watcher --ros-args \
  -p capture:=http \
  -p pickled:=True \
  -p topicId:=_http_py
```

Open another shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source ~/rosVid/ros2_ws/install/setup.bash
ros2 topic pub /captures/http/config_http_py std_msgs/String "data: '{height: 720, width: 1280}'" --once
sleep 1
ros2 topic pub /captures/http/config_http_py std_msgs/String "data: 'pubSpeed: 20'" --once
sleep 1
ros2 topic pub /captures/http/config_http_py std_msgs/String "data: '{height: 480, width: 640}'" --once
sleep 1
ros2 topic pub /captures/http/config_http_py std_msgs/String "data: 'pubSpeed: 1'" --once
```

With the above shell commands having been ran a watcher GUI should appear and have been manipulated accordingly.  If this test passes, proceed by closing out the shells.
