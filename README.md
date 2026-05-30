# rosVid
A software suite designed to simplify the complexities in streaming and receiving audio and video.

`rosVid` makes use of both [RTSP](https://datatracker.ietf.org/doc/html/rfc2326) and [ROS2](https://github.com/ros2) as transport and viewing mechanisms for video streams.

The supported types of input are:
 - HTTP
 - MIPI
 - RTSP
 - UDP
 - USB

 The supported types of output are:
 - ROS2 image viewing
 - RTSP
 
## Requirements
In the [SRC](SRC/) folder you will find [libStreamer](SRC/libStreamer/README.md) and [mavPool](SRC/mavPool/README.md).  These modules are designed to keep the code base slim by abstracting out things where it makes sense.

For ease of use both of these libraries have been pre-compiled under [Releases](https://github.com/stryngs/rosVid/releases).

While libStreamer is a requirement, mavPool has now been made optional and is invoked by the `mavPool` parameter.

For the ROS2 code, [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and [ROS2 Kilted](https://docs.ros.org/en/kilted/Installation.html) are both supported.

### rtspServer/rtspClient
The server and client are both stand-alone Python.  ROS2 is not required when `rtspServer` captures directly from a video device, USB camera, or MIPI camera.  ROS2 is required when `rtspServer` is run with `--capture topic`.

The RTSP server will transmit audio when the `--audio` flag is set.  To receive the audio with the RTSP client `--audio` must also be set.

Available parameters for server:
```
--abitrate   Opus bitrate                     [Default is 64000]
--acard      ALSA card index                  [Default is 0]
--achannels  Audio channels                   [Default is 2]
--adevice    ALSA deviceindex                 [Default is 0]
--arate      Audio sampel rate                [Default is 48000]
--audio      Enable audio                     [Default is False]
--bitrate    Desired bitrate                  [Default is 2500]
--capture    Capture source                   [Default is device, can also be topic]
--codec      Desired codec                    [Default is h264, can also be h265]
--device     Device to stream from            [Default is /dev/video0]
--endpoint   Desired endpoint for the stream  [Default is /video]
--format     Desired pixel format             [Default is I420]
--fps        Desired frames per second        [Default is 30]
--height     Desired video height             [Default is 480]
--key        UInt8MultiArray pickle image key [Default is frame]
--mipi       MIPI camera usage                [Default is none]
--port       Desired port to stream on        [Default is 8554]
--topic      ROS2 topic to stream             [Default is /captures/phys/pickled]
--type       ROS2 topic message type          [Default is UInt8MultiArray]
--width      Desired video width              [Default is 640]
```

When `--capture topic` is used, `--type` is case-sensitive and must match the ROS2 message family being subscribed:
```
UInt8MultiArray
CompressedImage
Image
```

For `UInt8MultiArray`, rosVid expects a pickled Python object containing JPEG bytes.  The default image key is `frame`, and it can be changed with `--key`.

Available parameters for client:
```
--audio              Enable audio                       [Default is False]
--audio-max-buffers  Audio queue max buffers            [Default is 5]
--audio-leaky        Leaky audio queue for low latency  [Default is True]
--audio-sync         Enable audio sink sync             [Default is True]
--codec              Video codec                        [Default is h264]
--drop-on-latency    Drop late buffers                  [Default is True]
--latency            RTSP source latency in ms          [Default is 0]
--video-leaky        Leaky video queue                  [Default is False]
--video-max-buffers  Video queue max buffers            [Default is 30]
--video-sync         Enable video sink sync             [Default is True]
--url                RTSP URL                           [Default is rtsp://127.0.0.1:8554/video]'
```

### HTTP
The captures module for HTTP allows connecting to a JPEG snapshot endpoint or MJPEG stream and then transmitting the contents over ROS2.  This is useful for browser-style camera feeds, lightweight dashboards, and other sources that expose frames over HTTP.

Example syntax:
```bash

ros2 run captures http
```

The above will connect to `http://127.0.0.1:8002/frame.jpg` by default.  To change this default behavior as well as other options you can choose from the available parameters below:
```
fps          Snapshot polling rate                      [Default is 10]
height       Expected / scaled video height             [Default is 480]
httpMode     HTTP input mode                            [Default is auto, can also be snapshot or mjpeg]
httpUrl      The associated HTTP URL to capture         [Default is http://127.0.0.1:8002/frame.jpg]
jpegQuality  JPEG compression quality                   [Default is 50]
mavPool      Enable MAVLink metadata integration        [Default is False]
pubSpeed     Published frames per second                [Default is 30]
timeout      HTTP request timeout in seconds            [Default is 5.0]
topicId      Defines a unique identifier for the topic  [Optional, for multiple instances]
width        Expected / scaled video width              [Default is 640]
```

When running multiple HTTP inputs at the same time the `topicId` parameter will modify the topics on the backend.

`httpMode` can be `auto`, `snapshot`, or `mjpeg`.  In `auto` mode the content type is inspected and multipart responses are treated as MJPEG; otherwise the URL is treated as a JPEG snapshot endpoint.

### MIPI
MIPI input is gathered by the [libStreamer.transports.rtsp](./SRC/libStreamer/libStreamer/transports/rtsp.py) module.

Example syntax:
```bash

python3 ./rtspServer.py --mipi
```

The above will connect to the MIPI camera on */dev/video0* by default.  If /dev/video0 is not the assignment for the MIPI camera, select the camera with `--device`.

The associated RTSP stream is now ready to view using more traditional RTSP viewing methods as well as the ROS2 viewing [technique](./README.md#watcher-output) further down in this file.

### RTSP
The captures module for RTSP allows connecting to an RTSP video stream and them transmitting the contents over ROS2.  If you are using a camera which already has RTSP capabilities the only thing to add is the `rtspUrl`.  If you are using the provided `rtspServer` then no parameters need to be added other than if you are doing this on two different computers, if so simply add the `rtspUrl` accordingly.

Example syntax:
```bash

ros2 run captures rtsp
```

On ROS2 Humble, if the `rtsp` capture node segfaults immediately after connecting, run it with Cyclone DDS:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run captures rtsp
```
This avoids a Humble-era `rclpy`/GStreamer crash seen with the default Fast DDS RMW on some systems.

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

**If using the h265 codec in conjunction with the `rtsp` module, that module must also invoke h265 via the codec parameter.**

### UDP
UDP input listens on port 5600 on all interfaces by default.

`udp` usage:
```bash

ros2 run captures udp
```

The available parameters for udp are as follows:
```
clockRate    RTP clock rate for the stream              [Default is 90000]
codec        Video codec (h264 or h265)                 [Default is h264]
height       Expected / scaled video height             [Default is 480]
jpegQuality  JPEG compression quality                   [Default is 50]
mavPool      Enable MAVLink metadata integration        [Default is False]
pubSpeed     Published frames per second                [Default is 30]
topicId      Defines a unique identifier for the topic  [Optional, for multiple instances]
udpHost      UDP source address to bind to              [Default is 0.0.0.0 (all interfaces)]
udpPort      UDP port to listen on                      [Default is 5600]
width        Expected / scaled video width              [Default is 640]
```

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

`rtspServer` can also restream a ROS2 image topic as RTSP by using `--capture topic`.  The RTSP pipeline receives decoded BGR frames from the ROS2 bridge, then encodes them as h264 or h265 for RTSP output.

Pickled `UInt8MultiArray` topic:
```bash
python3 ./rtspServer.py --capture topic \
  --topic /captures/phys/pickled \
  --type UInt8MultiArray \
  --key frame
```

Annotated `UInt8MultiArray` topic:
```bash
python3 ./rtspServer.py --capture topic \
  --topic /captures/phys/annotated \
  --type UInt8MultiArray \
  --key frame
```

Compressed image topic:
```bash
python3 ./rtspServer.py --capture topic \
  --topic /captures/phys/compressed \
  --type CompressedImage
```

Image topic:
```bash
python3 ./rtspServer.py --capture topic \
  --topic /captures/phys/raw \
  --type Image
```

Use `--width`, `--height`, and `--fps` to describe the RTSP output stream.  Frames from ROS2 topics are resized to the requested width and height before they are sent to RTSP.

### Watcher Output
The output produced by the `phys`, `http`, `rtsp`, or `udp` captures allows for a user to watch the video using the [watcher](ros2_ws/src/captures/captures/watcher.py) module.  The available parameters are as such:
```
capture     Defines which module to use                   [Default is phys, other options are http, rtsp, udp]
compressed  Defines whether to use the compressed options [Compression quality is determined via the phys or rtsp parameters]
key         UInt8MultiArray pickle image key             [Default is frame]
pickled     Defines whether to use the pickled approach   [Lowest latency]
topic       Exact ROS2 topic override                    [Optional]
topicId     Defines a unique identifier for the topic     [Optional, for multiple instances] 
type        Topic message type for topic override        [UInt8MultiArray, CompressedImage, Image]
```

If `capture` is not invoked the expected input is the `phys` module.

If neither `compressed` or `pickled` is invoked, then the [Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) message is using for receiving the message.

If `topicId` is used for phys, http, rtsp, or udp, it must be declared at launch for the watcher to capture the stream.

If `topic` is used, watcher subscribes to that exact topic.  Use `type` to select the decoder and `key` when the topic is a pickled `UInt8MultiArray`.

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

4. Freehand topic viewing:
```bash
ros2 run captures watcher --ros-args \
  -p topic:=/captures/phys/annotated \
  -p type:=UInt8MultiArray \
  -p key:=frame
```

## Configuration Changes
The `phys`, `http`, `rtsp` and `udp` executables create a ROS2 Subscriber which allows for user-defined changes while the module is running.  Detailed instructions for doing so are found [here](./ros2_ws/src/captures/README.md).
