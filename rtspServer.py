#!/usr/bin/env python3

import argparse
from libStreamer.transports.rtsp import *

def main():
    """Stream the RTSP"""
    parser = argparse.ArgumentParser(description = 'An RTSP based server')
    parser.add_argument('--bitrate', default = 2500, help = 'Desired bitrate [Default is 2500]')
    parser.add_argument('--codec', choices = ['h264', 'h265'], default = 'h264', help = 'Desired codec [Default is h264]')
    parser.add_argument('--device', default = '/dev/video0', help = 'Device to stream from [Default is /dev/video0]')
    parser.add_argument('--format', default = 'I420', help = 'Desired outbound pixel format [Default is I420]')
    parser.add_argument('--fps', default = '30', help = 'Desired frames per second [Default is 30]')
    parser.add_argument('--height', default = 480, help = 'Desired video height [Default is 480]')
    parser.add_argument('--mipi', action = 'store_true', help = 'MIPI camera usage [Default is none]')
    parser.add_argument('--port', default = 8554, help = 'Desired port to stream on [Default is 8554]')
    parser.add_argument('--endpoint', default = 'video', help = 'Desired endpoint for the stream [Default is /video]')
    parser.add_argument('--width', default = 640, help = 'Desired video width [Default is 640]')
 
    args = parser.parse_args()
    server = RTSPServer(args)

    print(f'[~] RTSP streaming on rtsp://0.0.0.0:{args.port}/{args.endpoint}')
    loop = GLib.MainLoop()
    loop.run()

if __name__ == '__main__':
    main()
