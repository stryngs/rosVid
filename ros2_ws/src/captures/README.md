# Captures
For portability the capture modules were created in such a way, even though ROS2 is needed, traditional methods of invoking `ros2 run` do not have to be used, rather a more Pythonic approach can be had.  Examples of how to do so are listed below.

## http
The http module was created to allow an HTTP JPEG snapshot or MJPEG stream to be transmitted over ROS2.

Invocation of the `http` module can be obtained without `ros2 run` being needed.  To set the various parameters you can provide arguments in the form of a dictionary when the class is instantiated:
```python
import rclpy
import threading
from captures.http import Http
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

rclpy.init()
node = Node('http')
http = Http(node, args = {'httpUrl': 'http://127.0.0.1:8002/frame.jpg',
                          'httpMode': 'snapshot',
                          'fps': 10})

executor = MultiThreadedExecutor()
executor.add_node(node)
theThread = threading.Thread(daemon = True, target = executor.spin)
theThread.start()
```

Incorporating the module into an already existing ROS2 stack can be done like so:
```python
from captures.http import Http

self.http = Http(self.node)
```

The available parameters that can be changed while running are:
- fps
- height
- httpMode
- httpUrl
- jpegQuality
- pubSpeed
- timeout
- width

`httpMode` can be `auto`, `snapshot`, or `mjpeg`.  In `auto` mode the content type is inspected and multipart responses are treated as MJPEG; otherwise the URL is treated as a JPEG snapshot endpoint.

config here
```bash
ros2 topic pub /captures/http/config std_msgs/String "data: '{httpUrl: \"http://127.0.0.1:8002/video.mjpg\", httpMode: mjpeg}'" --once
ros2 topic pub /captures/http/config std_msgs/String "data: 'pubSpeed: 20'" --once
```

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
- codec
- height
- jpegQuality
- pubSpeed
- rtspUrl
- width

config here
```bash
ros2 topic pub /captures/rtsp/config std_msgs/String "data: '{height: 720, width: 1280}'" --once
ros2 topic pub /captures/rtsp/config std_msgs/String "data: 'codec: h265'" --once
ros2 topic pub /captures/rtsp/config std_msgs/String "data: 'pubSpeed: 20'" --once
```

## udp
The udp module was created to allow for a UDP RTP stream (H.264/H.265) to be transmitted over ROS2.

Invocation of the `udp` module can be obtained without `ros2 run` being needed.  To set the various parameters you can provide arguments in the form of a dictionary when the class is instantiated.  Below is an example where the default pubSpeed setting is set to 15 and the port to 5600:
```python
import rclpy
import threading
from captures.udp import Udp
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

rclpy.init()
node = Node('udp')
udp = Udp(node, args = {'pubSpeed': 15, 'udpPort': 5600})

executor = MultiThreadedExecutor()
executor.add_node(node)
theThread = threading.Thread(daemon = True, target = executor.spin)
theThread.start()
```

Incorporating the module into an already existing ROS2 stack can be done like so:
```python
from captures.udp import Udp

self.udp = Udp(self.node)
```

The available parameters that can be changed while running are:
- clockRate
- codec
- height
- jpegQuality
- pubSpeed
- udpCaps
- udpHost
- udpPort
- width

config here
```bash
ros2 topic pub /captures/udp/config std_msgs/String "data: '{udpPort: 5600, codec: h264}'" --once
ros2 topic pub /captures/udp/config std_msgs/String "data: 'pubSpeed: 20'" --once
```

## Topic formats
The `phys`, `rtsp`, `udp`, and `http` modules publish the same three frame forms under their capture namespace:

| Topic suffix | ROS2 message type | Payload |
| --- | --- | --- |
| `/raw` | `sensor_msgs/msg/Image` | BGR image data |
| `/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG image data |
| `/pickled` | `std_msgs/msg/UInt8MultiArray` | Pickled Python object with JPEG bytes under `frame` |

For example, `phys` publishes:

```text
/captures/phys/raw
/captures/phys/compressed
/captures/phys/pickled
```

The same suffixes are available under `/captures/rtsp/...`, `/captures/udp/...`, and `/captures/http/...`.

## RTSP restreaming
The top-level [rtspServer](../../../rtspServer.py) can subscribe to any of these ROS2 topics and restream the frames over RTSP by using `--capture topic`.

Pickled topic:
```bash
python3 ../../../rtspServer.py --capture topic \
  --topic /captures/phys/pickled \
  --type UInt8MultiArray \
  --key frame
```

Compressed image topic:
```bash
python3 ../../../rtspServer.py --capture topic \
  --topic /captures/phys/compressed \
  --type CompressedImage
```

Image topic:
```bash
python3 ../../../rtspServer.py --capture topic \
  --topic /captures/phys/raw \
  --type Image
```

`--type` is case-sensitive and should match the ROS2 message type family: `UInt8MultiArray`, `CompressedImage`, or `Image`.  `--key` is only used for `UInt8MultiArray`; it defaults to `frame`.

## watcher
Information on interacting with the watcher executable is [here](../../../README.md#watcher-output).

By default watcher keeps the capture namespace behavior:

```bash
ros2 run captures watcher
```

The default subscribes to `/captures/phys/raw`.  The `compressed` and `pickled` parameters preserve the same priority used by the node:

```text
compressed=True -> /captures/<capture>/compressed<topicId>
pickled=True    -> /captures/<capture>/pickled<topicId>
else            -> /captures/<capture>/raw<topicId>
```

Examples:
```bash
ros2 run captures watcher --ros-args -p capture:=rtsp -p compressed:=true
ros2 run captures watcher --ros-args -p capture:=http -p pickled:=true
ros2 run captures watcher --ros-args -p capture:=udp
```

Watcher can also subscribe to a freehand topic by setting `topic`, `type`, and optionally `key`.  This matches the topic input style used by [rtspServer](../../../rtspServer.py):

```bash
ros2 run captures watcher --ros-args \
  -p topic:=/captures/phys/annotated \
  -p type:=UInt8MultiArray \
  -p key:=frame
```

Supported `type` values are `UInt8MultiArray`, `CompressedImage`, and `Image`.  `key` is only used for `UInt8MultiArray` pickled payloads and defaults to `frame`.
