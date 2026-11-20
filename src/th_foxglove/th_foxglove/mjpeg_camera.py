"""Republishes an MJPEG HTTP stream (e.g. the bench camera's uStreamer) as a ROS image topic.

The JPEG frames are passed through untouched (no decoding), so it costs almost no CPU:

  ~/url        http://localhost:8080/stream   uStreamer on the Jetson (tools/bench_camera)
  ~/topic      /camera/image/compressed       sensor_msgs/CompressedImage, format jpeg
  ~/frame_id   camera
  ~/max_rate   10.0 Hz; extra frames are dropped to spare Wi-Fi bandwidth
"""

import threading
import time
import urllib.request

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from th_foxglove.mjpeg import MjpegParser


class MjpegCamera(Node):

    def __init__(self):
        super().__init__('mjpeg_camera')
        self.url = self.declare_parameter('url', 'http://localhost:8080/stream').value
        topic = self.declare_parameter('topic', '/camera/image/compressed').value
        self.frame_id = self.declare_parameter('frame_id', 'camera').value
        max_rate = float(self.declare_parameter('max_rate', 10.0).value)
        # Degrees clockwise to turn frames upright (the walker's camera is mounted upside down: 180).
        # 0 passes the JPEGs through untouched; anything else decodes, rotates and re-encodes, so
        # every subscriber (dashboard, tag/person follower) gets the same upright image.
        self.rotation = int(self.declare_parameter('rotation', 0).value)
        if self.rotation not in (0, 90, 180, 270):
            raise ValueError('rotation must be 0, 90, 180 or 270')
        self.quality = int(self.declare_parameter('jpeg_quality', 85).value)
        self.period = 1.0 / max_rate if max_rate > 0 else 0.0
        self.publisher = self.create_publisher(CompressedImage, topic, qos_profile_sensor_data)
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Relaying {self.url} to {topic} at up to {max_rate:.0f} Hz')

    def run(self):
        connected_once = False
        while self.running and rclpy.ok():
            try:
                with urllib.request.urlopen(self.url, timeout=5.0) as stream:
                    if not connected_once:
                        self.get_logger().info('Camera stream connected')
                    connected_once = True
                    self.relay(stream)
            except Exception as error:  # network errors of every kind: retry
                self.get_logger().warn(
                    f'Camera stream {self.url} unavailable ({error}); retrying',
                    throttle_duration_sec=30.0)
            if self.running:
                time.sleep(2.0)

    def relay(self, stream):
        parser = MjpegParser()
        last = 0.0
        while self.running:
            chunk = stream.read1(65536) if hasattr(stream, 'read1') else stream.read(65536)
            if not chunk:
                return
            for jpeg in parser.feed(chunk):
                now = time.monotonic()
                if now - last < self.period:
                    continue
                last = now
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.format = 'jpeg'
                msg.data = self.rotate(jpeg) if self.rotation else jpeg
                self.publisher.publish(msg)

    def rotate(self, jpeg):
        import cv2  # only needed when rotating
        import numpy as np
        codes = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        image = cv2.imdecode(np.frombuffer(bytes(jpeg), np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return jpeg
        ok, out = cv2.imencode('.jpg', cv2.rotate(image, codes[self.rotation]),
                               [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return out.tobytes() if ok else jpeg

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MjpegCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
