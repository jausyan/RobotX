#!/usr/bin/env python3
"""
record_topic.py  –  record a ROS 2 image topic to .mp4
=======================================================
Subscribes to a sensor_msgs/Image topic and writes every frame to an .mp4.
The fps is measured from the first frames, so the video plays at real speed.

Usage:
    python3 script/record_topic.py                          # /camera/image -> script/video/topic_<time>.mp4
    python3 script/record_topic.py --topic /iris_with_camera_2/camera/image_raw
    python3 script/record_topic.py --fps 30                 # fixed fps instead of measuring
    python3 script/record_topic.py --duration 60            # stop after 60 s
    python3 script/record_topic.py --output out.mp4

Press Ctrl+C to stop (the file is closed properly).
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FPS_PROBE_FRAMES = 30   # frames used to measure the topic rate


def image_to_bgr(msg):
    """sensor_msgs/Image -> BGR numpy array (no cv_bridge needed)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img if enc == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc in ('bgra8', 'rgba8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == 'bgra8' else cv2.COLOR_RGBA2BGR)
    if enc in ('mono8', '8uc1'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc in ('yuv422', 'uyvy'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 2].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)
    if enc in ('yuv422_yuy2', 'yuyv'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 2].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    raise ValueError(f'unsupported encoding: {msg.encoding}')


class TopicRecorder(Node):
    def __init__(self, topic, output, fps, duration):
        super().__init__('record_topic')
        self.output = output
        self.fixed_fps = fps
        self.duration = duration
        self.writer = None
        self.frame_size = None
        self.probe = []            # (stamp, frame) until the fps is known
        self.frames = 0
        self.start = None
        self.done = False
        self.create_subscription(Image, topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(f'Waiting for images on {topic} ...')

    def open_writer(self, fps, size):
        self.frame_size = size
        self.writer = cv2.VideoWriter(self.output, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        if not self.writer.isOpened():
            raise RuntimeError(f'cannot open {self.output} for writing')
        self.get_logger().info(f'Recording {size[0]}x{size[1]} @ {fps:.1f} fps -> {self.output}')

    def callback(self, msg):
        if self.done:
            return
        try:
            frame = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if self.start is None:
            self.start = now
        if self.duration > 0 and now - self.start >= self.duration:
            self.get_logger().info(f'Duration {self.duration:.0f} s reached')
            self.done = True
            return

        if self.writer is None:
            size = (frame.shape[1], frame.shape[0])
            if self.fixed_fps > 0:
                self.open_writer(self.fixed_fps, size)
            else:
                # measure the real topic rate first, so playback is not slow/fast motion
                self.probe.append((now, frame))
                if len(self.probe) < FPS_PROBE_FRAMES:
                    return
                span = self.probe[-1][0] - self.probe[0][0]
                fps = (len(self.probe) - 1) / span if span > 0 else 30.0
                self.open_writer(fps, size)
                for _, f in self.probe:
                    self.write(f)
                self.probe = []
                return
        self.write(frame)

    def write(self, frame):
        h, w = frame.shape[:2]
        if (w, h) != self.frame_size:
            frame = cv2.resize(frame, self.frame_size)
        self.writer.write(frame)
        self.frames += 1
        if self.frames % 100 == 0:
            self.get_logger().info(f'{self.frames} frames written')

    def close(self):
        if self.writer is None and self.probe:
            # stopped before the fps was measured: write what we have at 30 fps
            f = self.probe[0][1]
            self.open_writer(self.fixed_fps or 30.0, (f.shape[1], f.shape[0]))
            for _, f in self.probe:
                self.write(f)
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f'Saved {self.frames} frames -> {self.output}')
        else:
            self.get_logger().warn('No frames received, nothing saved')


def main():
    parser = argparse.ArgumentParser(description='Record a ROS 2 image topic to .mp4')
    parser.add_argument('--topic', default='/camera/image')
    parser.add_argument('--output', default='', help='default: script/video/topic_<time>.mp4')
    parser.add_argument('--fps', type=float, default=0.0, help='0 = measure from the topic')
    parser.add_argument('--duration', type=float, default=0.0, help='seconds, 0 = until Ctrl+C')
    args = parser.parse_args()

    output = args.output
    if not output:
        os.makedirs(os.path.join(SCRIPT_DIR, 'video'), exist_ok=True)
        output = os.path.join(SCRIPT_DIR, 'video', f'topic_{datetime.now():%Y%m%d_%H%M%S}.mp4')

    rclpy.init()
    node = TopicRecorder(args.topic, output, args.fps, args.duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
