#!/usr/bin/env python3
"""
ROS2 Camera Image Subscriber and Viewer
Subscribes to /iris_with_camera_2/camera/image_raw and displays the stream.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
from collections import deque


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        self.window_name = 'Iris Camera - /iris_with_camera_2/camera/image_raw'

        # FPS tracking: rolling average over last 30 frames
        self._frame_times: deque = deque(maxlen=30)
        self._fps: float = 0.0

        # Frame capture
        self._save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'record')
        os.makedirs(self._save_dir, exist_ok=True)
        self._capture_count: int = 0

        # Video recording
        self._video_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'video')
        os.makedirs(self._video_dir, exist_ok=True)
        self._video_writer: cv2.VideoWriter | None = None
        self._is_recording: bool = False
        self._video_path: str = ''
        self._rec_blink_state: bool = True
        self._rec_blink_counter: int = 0

        self.subscription = self.create_subscription(
            Image,
            '/iris_with_camera_2/camera/image_raw',
            self.image_callback,
            10  # QoS history depth
        )

        self.get_logger().info('Camera viewer node started.')
        self.get_logger().info('Subscribing to: /iris_with_camera_2/camera/image_raw')
        self.get_logger().info('Press "c" to capture a frame  |  Press "v" to toggle video recording  |  Press "q" to quit.')
        self.get_logger().info(f'Frame captures : {self._save_dir}')
        self.get_logger().info(f'Video recordings: {self._video_dir}')

    def image_callback(self, msg: Image):
        """Convert ROS Image message to OpenCV image and display it."""
        try:
            # Convert ROS Image message to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # FPS calculation using rolling window
        now = time.time()
        self._frame_times.append(now)
        if len(self._frame_times) >= 2:
            elapsed = self._frame_times[-1] - self._frame_times[0]
            self._fps = (len(self._frame_times) - 1) / elapsed if elapsed > 0 else 0.0

        # Display image dimensions
        h, w = cv_image.shape[:2]
        self.get_logger().debug(f'Received frame: {w}x{h} @ {self._fps:.1f} FPS')

        # Write frame to video if recording
        if self._is_recording and self._video_writer is not None:
            self._video_writer.write(cv_image)

        # Overlay resolution + encoding
        info_text = f'{w}x{h} | {msg.encoding}'
        cv2.putText(
            cv_image, info_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65,
            (0, 255, 0), 2, cv2.LINE_AA
        )

        # Overlay FPS
        fps_text = f'FPS: {self._fps:.1f}'
        cv2.putText(
            cv_image, fps_text,
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65,
            (0, 200, 255), 2, cv2.LINE_AA
        )

        # Blinking REC indicator when recording
        if self._is_recording:
            self._rec_blink_counter += 1
            if self._rec_blink_counter % 20 == 0:  # toggle every ~20 frames
                self._rec_blink_state = not self._rec_blink_state
            if self._rec_blink_state:
                cv2.circle(cv_image, (w - 30, 20), 10, (0, 0, 255), -1)
                cv2.putText(
                    cv_image, 'REC',
                    (w - 85, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 0, 255), 2, cv2.LINE_AA
                )

        cv2.imshow(self.window_name, cv_image)

        # Key handling
        key = cv2.waitKey(1) & 0xFF

        # 'c' / 'C' — capture frame
        if key in (ord('c'), ord('C')):
            self._save_frame(cv_image)

        # 'v' / 'V' — toggle video recording
        elif key in (ord('v'), ord('V')):
            if not self._is_recording:
                self._start_recording(cv_image)
            else:
                self._stop_recording()

        # 'q' / 'Q' — quit
        elif key in (ord('q'), ord('Q')):
            self.get_logger().info('Quit key pressed. Shutting down...')
            rclpy.shutdown()

    def _save_frame(self, frame):
        """Save the current frame as a timestamped PNG to self._save_dir."""
        self._capture_count += 1
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = f'frame_{timestamp}_{self._capture_count:04d}.png'
        filepath = os.path.join(self._save_dir, filename)
        success = cv2.imwrite(filepath, frame)
        if success:
            self.get_logger().info(f'[Capture #{self._capture_count}] Saved: {filepath}')
        else:
            self.get_logger().error(f'Failed to save frame to: {filepath}')

    def _start_recording(self, frame):
        """Initialize VideoWriter and start recording."""
        h, w = frame.shape[:2]
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = f'video_{timestamp}.mp4'
        self._video_path = os.path.join(self._video_dir, filename)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = self._fps if self._fps > 0 else 30.0
        self._video_writer = cv2.VideoWriter(self._video_path, fourcc, fps, (w, h))
        if self._video_writer.isOpened():
            self._is_recording = True
            self._rec_blink_counter = 0
            self._rec_blink_state = True
            self.get_logger().info(f'[REC] Started recording: {self._video_path}  ({w}x{h} @ {fps:.1f} FPS)')
        else:
            self._video_writer = None
            self.get_logger().error(f'Failed to open VideoWriter for: {self._video_path}')

    def _stop_recording(self):
        """Finalize and release the VideoWriter."""
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        self._is_recording = False
        self.get_logger().info(f'[REC] Stopped. Video saved: {self._video_path}')


def main(args=None):
    rclpy.init(args=args)

    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user (Ctrl+C). Shutting down...')
    finally:
        if node._is_recording:
            node._stop_recording()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
