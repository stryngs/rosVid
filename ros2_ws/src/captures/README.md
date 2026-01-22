# Captures
For portability the `phys` and `rtsp` modules were created in such a way, even though ROS2 is needed, traditional methods of invoking `ros2 run` do not have to be used, rather a more Pythonic approach can be had.  Examples of how to do so are listed below.

## phys
The phys module was created to allow for a physically connected camera stream to be transmitted over ROS2.  It supports things like Webcams and USB cameras.

Invocation of the `phys` module can be obtained without `ros2 run` being needed.  To set the various parameters you can provide arguments in the form of a dictionary when the class is instantiated.  Below is an example where the default pubSpeed setting is set to 15 and not the default of 30:
```python
import rclpy
import threading
from libStreamer_phys import Phys
from rclpy.node import Node

rclpy.init()
node = Node('phys')
phy = Phys(node, args = {'pubSpeed': 15})

theThread = threading.Thread(daemon = True, target = lambda: rclpy.spin(node))
theThread.start()
```

Incorporating the module into an already existing ROS2 stack can be done like so:
```python
from libStreamer_phys import Phys

self.phy = Phys(self.node)
```

The available parameters that can be changed while running are:
- device
- fps
- height
- jpegQuality
- pubSpeed
- width

Changing of a parameter's value is accomplished by publishing to a ROS2 topic.  An example of doing so is:
```bash
ros2 topic pub /captures/rtsp/config std_msgs/String "data: '{height: 720, width: 1280}'" --once
ros2 topic pub /captures/rtsp/config std_msgs/String "data: 'pubSpeed: 20'" --once
```

## rtsp
The rtsp module was created to allow for an RTSP stream to be transmitted over ROS2.

Invocation of the `rtsp` module can be obtained without `ros2 run` being needed.  To set the various parameters you can provide arguments in the form of a dictionary when the class is instantiated.  Below is an example where the default pubSpeed setting is set to 15 and not the default of 30:
```python
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
```

Incorporating the module into an already existing ROS2 stack can be done like so:
```python
from libStreamer_rtsp import Rtsp

self.rts = Rtsp(self.node)
```

The available parameters that can be changed while running are:
- height
- jpegQuality
- pubSpeed
- rtspUrl
- width

config here
```bash
ros2 topic pub /captures/rtsp/config std_msgs/String "data: '{height: 720, width: 1280}'" --once
ros2 topic pub /captures/rtsp/config std_msgs/String "data: 'pubSpeed: 20'" --once
```

## watcher
Information on interacting with the watcher executable is [here](../../../README.md#watcher-output).
