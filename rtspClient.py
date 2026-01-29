#!/usr/bin/env python3

import argparse
import gi
import sys

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

def mHandler(bus, message, loop):
    msg_type = message.type
    if msg_type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f'[!] GST error {err.message}')
        if debug:
            print(f'[~] debug {debug}')
        loop.quit()
    elif msg_type == Gst.MessageType.EOS:
        print('[~] EOS')
        loop.quit()
    return True


def pipebuild(url, latency, drop_on_latency, video_leaky, audio_leaky,
                   video_max_buffers, audio_max_buffers, video_sync, audio_sync, codec, audio):
    """RTSP pipeline builder handling audio and video"""
    drop_val = 'true' if drop_on_latency else 'false'
    v_leaky = 'downstream' if video_leaky else 'no'
    a_leaky = 'downstream' if audio_leaky else 'no'
    v_queue_props = f'max-size-buffers={video_max_buffers} max-size-time=0 max-size-bytes=0 leaky={v_leaky}'
    a_queue_props = f'max-size-buffers={audio_max_buffers} max-size-time=0 max-size-bytes=0 leaky={a_leaky}'

    vchain = (f'queue {v_queue_props} ! '
              f'application/x-rtp,media=video,payload=96 ! '
              f'rtp{codec}depay ! {codec}parse ! avdec_{codec} ! videoconvert ! '
              f'autovideosink sync={str(video_sync).lower()}')

    achain = (f'queue {a_queue_props} ! '
              f'application/x-rtp,media=audio,encoding-name=OPUS,payload=97,clock-rate=48000 ! '
              f'rtpopusdepay ! opusdec ! audioconvert ! audioresample ! '
              f'autoaudiosink sync={str(audio_sync).lower()}')

    if audio:
        pipeline_str = (f'rtspsrc location={url} latency={latency} drop-on-latency={drop_val} name=src '
                        f'src. ! {vchain} '
                        f'src. ! {achain}')
    else:
        pipeline_str = (f'rtspsrc location={url} latency={latency} drop-on-latency={drop_val} name=src '
                        f'src. ! {vchain}')

    return Gst.parse_launch(pipeline_str)


def main():
    parser = argparse.ArgumentParser(description = 'An RTSP based client')

    parser.add_argument('--audio',
                        action = 'store_true',
                        help = 'Enable audio branch [Default is False]')
    parser.add_argument('--audio-max-buffers',
                        default = 5,
                        help = 'Audio queue max buffers [Default is 5]',
                        type = int)
    parser.add_argument('--audio-leaky',
                        action = argparse.BooleanOptionalAction,
                        default = True,
                        help = 'Leaky audio queue for low latency [Default is True]')
    parser.add_argument('--audio-sync',
                        action = argparse.BooleanOptionalAction, default = True,
                        help = 'Enable audio sink sync [Default is True]')
    parser.add_argument('--codec',
                        choices = ['h264', 'h265'],
                        default = 'h264',
                        help = 'Video codec [Default is h264]')
    parser.add_argument('--drop-on-latency',
                        action = argparse.BooleanOptionalAction,
                        default = True,
                        help = 'Drop late buffers [Default is True]')
    parser.add_argument('--latency',
                        default = 0,
                        help = 'rtspsrc latency in ms [Default is 0]',
                        type = int)
    parser.add_argument('--video-leaky',
                        action = argparse.BooleanOptionalAction,
                        default = False,
                        help = 'Leaky video queue (can cause green frames) [Default is False]')
    parser.add_argument('--video-max-buffers',
                        default = 30,
                        help = 'Video queue max buffers [Default is 30]',
                        type = int)
    parser.add_argument('--video-sync',
                        action = argparse.BooleanOptionalAction,
                        default = True,
                        help = 'Enable video sink sync [Default is True]')
    parser.add_argument('--url',
                        default = 'rtsp://127.0.0.1:8554/video',
                        help = 'RTSP URL [Default is rtsp://127.0.0.1:8554/video]')
    args = parser.parse_args()

    try:
        player = pipebuild(args.url,
                                args.latency,
                                args.drop_on_latency,
                                args.video_leaky,
                                args.audio_leaky,
                                args.video_max_buffers,
                                args.audio_max_buffers,
                                args.video_sync,
                                args.audio_sync,
                                args.codec,
                                args.audio)
    except Exception as E:
        print(f'[!] Failed to build pipeline: {E}')
        return 1

    bus = player.get_bus()
    bus.add_signal_watch()
    loop = GLib.MainLoop()
    bus.connect('message', mHandler, loop)

    player.set_state(Gst.State.PLAYING)
    print(f'[~] RTSP playing {args.url}')

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        player.set_state(Gst.State.NULL)
        bus.remove_signal_watch()
    return 0

if __name__ == '__main__':
    sys.exit(main())
