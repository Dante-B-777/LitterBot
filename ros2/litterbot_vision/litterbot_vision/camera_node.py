import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_node')

        self.declare_parameter('device', 4)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self._device = int(self.get_parameter('device').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._fps = float(self.get_parameter('fps').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        self._image_pub = self.create_publisher(CompressedImage, '/camera/image/compressed', 2)
        self._status_pub = self.create_publisher(Bool, '/camera/status', 2)
        self._info_pub = self.create_publisher(String, '/camera/info', 10)

        self._cap = None
        self._preferred_devices = [self._device, 4, 2, 0]
        self._open_camera()

        period = 1.0 / max(self._fps, 0.5)
        self._last_info_warn = 0.0
        self.create_timer(period, self._tick)

    def _open_camera(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

        tried = set()
        for device in self._preferred_devices:
            if device in tried:
                continue
            tried.add(device)

            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            cap.set(cv2.CAP_PROP_FPS, self._fps)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            ok, _ = cap.read()
            if ok:
                self._cap = cap
                self._device = device
                self.get_logger().info(
                    f'camera_node opened /dev/video{self._device} at '
                    f'{self._width}x{self._height} @{self._fps:.1f} fps (MJPG)'
                )
                return
            cap.release()

        self.get_logger().error('camera_node failed to open any candidate /dev/video device')

    def _tick(self) -> None:
        ok = self._cap is not None and self._cap.isOpened()
        if ok:
            ok, frame = self._cap.read()

        status = Bool()
        status.data = bool(ok)
        self._status_pub.publish(status)

        if not ok:
            now = time.monotonic()
            if now - self._last_info_warn > 5.0:
                self._last_info_warn = now
                self._info_pub.publish(String(data='camera read failed'))
            self._open_camera()
            return

        encoded, jpeg = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not encoded:
            self._info_pub.publish(String(data='camera jpeg encode failed'))
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()
        self._image_pub.publish(msg)

    def destroy_node(self) -> bool:
        if hasattr(self, '_cap') and self._cap is not None:
            self._cap.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
