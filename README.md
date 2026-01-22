# rosVid
A software suite designed to simplify the complexities in streaming and receiving a video image.

`rosVid` makes use of both [RTSP](https://datatracker.ietf.org/doc/html/rfc2326) and [ROS2](https://github.com/ros2) as transport and viewing mechanisms for video streams.

The supported types of input are:
 - MIPI
 - RTSP
 - USB

 The supported types of output are:
 - ROS2 image viewing
 - RTSP
 
## Requirements
In the [SRC](SRC/) folder you will find [libStreamer](SRC/libStreamer/README.md) and [mavPool](SRC/mavPool/README.md).  These modules are designed to keep the code base slim by abstracting out things where it makes sense.

For ease of use both of these libraries have been pre-compiled under [Releases](https://github.com/stryngs/rosVid/releases).

The `rtspServer` is stand-alone Python and no ROS2 installation is required.

Usage of the captures package requires ROS2.  [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and [ROS2 Kilted](https://docs.ros.org/en/kilted/Installation.html) are both supported.

## Inputs
Physical cameras via USB or MIPI, as well as RTSP as an input, is achieved using the methods below.  The input mechanism for [rtspServer](./rtspServer.py) allows either a USB or MIPI camera to be captured and streamed as RTSP.

### MIPI
MIPI input is gathered by the [libStreamer.transports.rtsp](./SRC/libStreamer/libStreamer/transports/rtsp.py) module.

Example syntax:
```bash

python3 ./rtspServer.py --mipi
```

The above will connect to the MIPI camera on */dev/video0* by default.  If /dev/video0 is not the assignment for the MIPI camera, select the camera with `--device`.

The associated RTSP stream is now ready to view using more traditional RTSP viewing methods as well as the ROS2 viewing [technique](./README.md#watcher-output) further down in this file.

**If using the h265 codec in conjunction with the `rtsp` module, that module must also invoke h265 via the codec parameter.**

Available parameters:
```
  --bitrate BITRATE    Desired bitrate                  [Default is 2500]
  --codec {h264,h265}  Desired codec                    [Default is h264]
  --device DEVICE      Device to stream from            [Default is /dev/video0]
  --format FORMAT      Desired pixel format             [Default is YUY2]
  --fps FPS            Desired frames per second        [Default is 30]
  --height HEIGHT      Desired video height             [Default is 480]
  --mipi               MIPI camera usage                [Default is none]
  --port PORT          Desired port to stream on        [Default is 8554]
  --endpoint ENDPOINT  Desired endpoint for the stream  [Default is /video]
  --width WIDTH        Desired video width              [Default is 640]
```

### RTSP
The captures module for RTSP allows connecting to an RTSP video stream and them transmitting the contents over ROS2.  If you are using a camera which already has RTSP capabilities the only thing to add is the `rtspUrl`.  If you are using the provided `rtspServer` then no parameters need to be added other than if you are doing this on two different computers, if so simply add the `rtspUrl` accordingly.

Example syntax:
```bash

ros2 run captures rtsp
```

The above will connect to an RTSP stream running on `rtsp://127.0.0.1:8554/video`.  To change this default behavior as well as other options you can choose from the available parameters below:
```
codec        h264 or h265                               [Default is h264]
height       Height of video to output                  [Default is 480]
jpegQuality  Compression quality                        [Default is 50]
pubSpeed     Rate of published frames per second        [Default is 30]
rtspUrl      The associated RTSP URL to capture         [Default is rtsp://127.0.0.1:8554/video]
topicId      Defines a unique identifier for the topic  [Optional, for multiple instances] 
width        Width of video to output                   [Default is 640]
```

When running multiple video cameras at the same time the `topicId` parameter will modify the topics on the backend.

### USB
USB input is gathered using either the [phys](ros2_ws/src/captures/captures/phys.py) module or the [rtspServer](./rtspServer.py).

Connection using the `phys` module allows for transmission of the video over ROS2.

Connection using the `rtspServer` allows for transmission of the video over RTSP as well as being able to be transmitted over ROS2 via the [rtsp](ros2_ws/src/captures/captures/rtsp.py) module.

`phys` usage:
```bash

ros2 run captures phys
```

The available parameters for phys are as follows:
```
device       The device to use                          [Default is /dev/video0]
fps          Transmitted frames per second              [Default is 30]
height       The transmitted video height               [Default is 480]
jpegQuality  Compression quality                        [Default is 50]
pubSpeed     Published frames per second                [Default is 30]
topicId      Defines a unique identifier for the topic  [Optional, for multiple instances] 
width        The transmitted video width                [Default is 640]
```

`rtspServer` usage:
```bash

python3 ./rtspServer.py
```

When running multiple video cameras at the same time the `topicId` parameter will modify the topics on the backend.

### RTSP Output
The output produced by `rtspServer` allows for a user to stream the video to an RTSP viewer of their choice.

### Watcher Output
The output produced by the `phys` or `rtsp` captures allows for a user to watch the video using the [watcher](ros2_ws/src/captures/captures/watcher.py) module.  The available parameters are as such:
```
capture     Defines which module to use                   [Default is phys, other option is rtsp]
compressed  Defines whether to use the compressed options [Compression quality is determined via the phys or rtsp parameters]
pickled     Defines whether to use the pickled approach   [Lowest latency]
topicId     Defines a unique identifier for the topic     [Optional, for multiple instances] 
```

If `capture` is not invoked the expected input is the `phys` module.

If neither `compressed` or `pickled` is invoked, then the [Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) message is using for receiving the message.

If `topicId` is used for either phys or rtsp, it must be declared at launch for the watcher to capture the stream.

In terms of which approach to use, "it depends":
1. The pickled approach is the smoothest in terms of quality and latency, very close to an FPV type feel:
```bash
ros2 run captures watcher --ros-args -p pickled:=True
```

2. The compressed viewing is better in terms of quality and latency than `Image`:
```bash
ros2 run captures watcher --ros-args -p compressed:=True -p capture:=rtsp
```

3. Image viewing:
```bash
ros2 run captures watcher
```

## Configuration Changes
Both the `phys` and `rtsp` executables create a ROS2 Subscriber which allows for user-defined changes while the module is running.  Detailed instructions for doing so are found [here](./ros2_ws/src/captures/README.md).

## Next steps
In the next phase of development, rosVid will evolve from a streaming and viewing platform into a reactive system.  The idea is to monitor the pixel-level changes from the video feed, detect motion, identify patterns, and trigger automated responses in real time (hence ROS2 from the beginning).  That capability will open the door for applications ranging from surveillance and robotics to autonomous vehicles, where instantaneous visual feedback is critical.  The focus will be on creating a modular framework that allows users to define custom reactions to visual events, transforming raw video streams into actionable intelligence.
