# phys Unit Testing
The following instructions have been developed for regression testing of the rtsp module.


## Python test
With the modPool module installed in your Python environment, SITL running, and a Mavros node online, open a shell and do:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
python3 << 'EOF'
import rclpy
import threading
import time
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
time.sleep(3)
print(mvp._gps)
print(mvp._imu)
executor.shutdown()
node.destroy_node()
EOF
```

If GPS and IMU data is displayed, the test has passed.
