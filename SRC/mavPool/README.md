# mavPool
A class where you query a dictionary instead of having to subscribe to a mavros topic.

The `mavPool` module does not use the traditional `ros2 run` approach:
```python
import rclpy
import threading
from mavPool import Pool
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

rclpy.init()
node = Node('mavPool')
mvp = Pool(node)
executor = MultiThreadedExecutor()
executor.add_node(node)
theThread = threading.Thread(daemon = True, target = executor.spin)
theThread.start()
```

Incorporating the module into an already existing ROS2 stack can be done like so:
```python
from mavPool import Pool

self.mvp = Pool(self.node)
```
