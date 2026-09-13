#!/usr/bin/env python3
"""Publish the A2 front RTP/H.264 multicast stream as throttled JPEG images."""

from __future__ import annotations

import math
import threading
import time

import cv2
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FrontCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("front_camera_bridge")

        self._multicast_group = str(
            self.declare_parameter("multicast_group", "230.1.1.1").value
        )
        self._udp_port = int(self.declare_parameter("udp_port", 1720).value)
        self._network_interface = str(
            self.declare_parameter("network_interface", "eth0").value
        )
        self._output_topic = str(
            self.declare_parameter(
                "output_topic", "/camera/front/image/compressed"
            ).value
        )
        self._frame_id = str(self.declare_parameter("frame_id", "camera_link").value)
        self._output_width = int(self.declare_parameter("output_width", 640).value)
        self._output_height = int(self.declare_parameter("output_height", 360).value)
        self._jpeg_quality = int(self.declare_parameter("jpeg_quality", 75).value)
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 15.0).value
        )
        self._jitter_latency_ms = int(
            self.declare_parameter("jitter_latency_ms", 20).value
        )
        self._stale_timeout_sec = float(
            self.declare_parameter("stale_timeout_sec", 3.0).value
        )

        if not self._multicast_group or not self._network_interface:
            raise ValueError("multicast group and network interface must not be empty")
        if not 1 <= self._udp_port <= 65535:
            raise ValueError("udp_port must be in [1, 65535]")
        if not self._output_topic or not self._frame_id:
            raise ValueError("output topic and frame ID must not be empty")
        if self._output_width <= 0 or self._output_height <= 0:
            raise ValueError("output dimensions must be positive")
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")
        if not math.isfinite(self._publish_rate_hz) or not 0 < self._publish_rate_hz <= 30:
            raise ValueError("publish_rate_hz must be in (0, 30]")
        if not 0 <= self._jitter_latency_ms <= 500:
            raise ValueError("jitter_latency_ms must be in [0, 500]")
        if not math.isfinite(self._stale_timeout_sec) or self._stale_timeout_sec <= 0:
            raise ValueError("stale_timeout_sec must be finite and positive")

        self._publisher = self.create_publisher(
            CompressedImage, self._output_topic, qos_profile_sensor_data
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest_jpeg: bytes | None = None
        self._latest_sequence = 0
        self._published_sequence = 0
        self._last_frame_monotonic = 0.0
        self._capture_open = False
        self._captured_frames = 0
        self._published_frames = 0
        self._decode_failures = 0
        self._encode_failures = 0
        self._open_failures = 0

        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="a2-front-camera", daemon=True
        )
        self._capture_thread.start()
        # Poll more frequently than frames are encoded. Publication remains
        # bounded by the encoded-frame sequence, while timer phase contributes
        # at most about 1/4 frame instead of one complete frame of latency.
        self._publish_timer = self.create_timer(
            0.25 / self._publish_rate_hz, self._publish_latest
        )
        self._diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            "Receiving read-only A2 front RTP/H.264 %s:%d on %s; publishing "
            "%dx%d JPEG to %s at up to %.1f Hz"
            % (
                self._multicast_group,
                self._udp_port,
                self._network_interface,
                self._output_width,
                self._output_height,
                self._output_topic,
                self._publish_rate_hz,
            )
        )

    def _pipeline(self) -> str:
        return (
            f"udpsrc address={self._multicast_group} port={self._udp_port} "
            f"multicast-iface={self._network_interface} buffer-size=1048576 ! "
            "application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            f"rtpjitterbuffer latency={self._jitter_latency_ms} "
            "drop-on-latency=true do-lost=true ! "
            "rtph264depay ! h264parse ! avdec_h264 max-threads=1 ! "
            "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
            "leaky=downstream ! videoconvert n-threads=2 ! "
            "videoscale method=2 n-threads=2 ! "
            f"video/x-raw,width={self._output_width},height={self._output_height},"
            "format=BGR ! appsink drop=true max-buffers=1 sync=false "
            "enable-last-sample=false"
        )

    def _capture_loop(self) -> None:
        encode_parameters = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        # Encode only frames that can be published. The leaky GStreamer queues
        # and appsink retain the newest decoded frame instead of accumulating
        # latency when CPU or the SSH/Foxglove link is briefly busy.
        encode_interval = 1.0 / self._publish_rate_hz
        while not self._stop.is_set():
            capture = cv2.VideoCapture(self._pipeline(), cv2.CAP_GSTREAMER)
            with self._lock:
                self._capture_open = capture.isOpened()
                if not self._capture_open:
                    self._open_failures += 1
            if not capture.isOpened():
                capture.release()
                self._stop.wait(2.0)
                continue

            next_encode_monotonic = 0.0
            while not self._stop.is_set():
                success, frame = capture.read()
                if not success or frame is None:
                    with self._lock:
                        self._decode_failures += 1
                    break
                now = time.monotonic()
                if now < next_encode_monotonic:
                    continue
                next_encode_monotonic = now + encode_interval
                encoded, buffer = cv2.imencode(".jpg", frame, encode_parameters)
                if not encoded:
                    with self._lock:
                        self._encode_failures += 1
                    continue
                jpeg = buffer.tobytes()
                with self._lock:
                    self._latest_jpeg = jpeg
                    self._latest_sequence += 1
                    self._last_frame_monotonic = time.monotonic()
                    self._captured_frames += 1

            capture.release()
            with self._lock:
                self._capture_open = False
            self._stop.wait(0.5)

    def _publish_latest(self) -> None:
        with self._lock:
            if (
                self._latest_jpeg is None
                or self._latest_sequence == self._published_sequence
            ):
                return
            jpeg = self._latest_jpeg
            sequence = self._latest_sequence

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.format = "jpeg"
        message.data = jpeg
        self._publisher.publish(message)

        with self._lock:
            self._published_sequence = sequence
            self._published_frames += 1

    @staticmethod
    def _value(key: str, value: object) -> KeyValue:
        return KeyValue(key=key, value=str(value))

    def _publish_diagnostics(self) -> None:
        with self._lock:
            age = (
                -1.0
                if self._last_frame_monotonic == 0.0
                else time.monotonic() - self._last_frame_monotonic
            )
            capture_open = self._capture_open
            captured_frames = self._captured_frames
            published_frames = self._published_frames
            decode_failures = self._decode_failures
            encode_failures = self._encode_failures
            open_failures = self._open_failures
            payload_bytes = len(self._latest_jpeg) if self._latest_jpeg else 0

        status = DiagnosticStatus()
        status.name = "u_robot_camera_bridge/front_camera"
        status.hardware_id = "a2-pro/front-rtp-camera"
        if age < 0:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for A2 front-camera RTP/H.264"
        elif age > self._stale_timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "A2 front-camera stream is stale"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "A2 front-camera JPEG healthy"
        status.values = [
            self._value("multicast_group", self._multicast_group),
            self._value("udp_port", self._udp_port),
            self._value("network_interface", self._network_interface),
            self._value("jitter_latency_ms", self._jitter_latency_ms),
            self._value("output_topic", self._output_topic),
            self._value("capture_open", capture_open),
            self._value("captured_frames", captured_frames),
            self._value("published_frames", published_frames),
            self._value("last_payload_bytes", payload_bytes),
            self._value("last_frame_age_sec", f"{age:.3f}"),
            self._value("open_failures", open_failures),
            self._value("decode_failures", decode_failures),
            self._value("encode_failures", encode_failures),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_publisher.publish(array)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._capture_thread.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: FrontCameraBridge | None = None
    try:
        node = FrontCameraBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # ROS 2 Humble's SIGINT handler may already have shut down the default
        # context before control reaches this block.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
