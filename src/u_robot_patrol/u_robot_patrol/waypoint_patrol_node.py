#!/usr/bin/env python3
"""Persist operator-selected map poses and execute them through Nav2."""

import json
import math
import os
from pathlib import Path
import tempfile
from typing import Callable, Optional

import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Empty as EmptyService, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class WaypointPatrolNode(Node):
    """Own waypoint persistence and sequential NavigateToPose action goals."""

    def __init__(self) -> None:
        super().__init__("waypoint_patrol")
        self._waypoint_topic = self.declare_parameter(
            "waypoint_topic", "/operator/waypoint"
        ).value
        self._command_topic = self.declare_parameter(
            "command_topic", "/operator/patrol_command"
        ).value
        self._cancel_topic = self.declare_parameter(
            "cancel_topic", "/operator/cancel_navigation"
        ).value
        self._marker_topic = self.declare_parameter(
            "marker_topic", "/navigation/waypoints"
        ).value
        self._active_topic = self.declare_parameter(
            "active_topic", "/waypoint_patrol/active"
        ).value
        self._status_topic = self.declare_parameter(
            "status_topic", "/waypoint_patrol/status"
        ).value
        self._map_topic = self.declare_parameter("map_topic", "/map").value
        self._action_name = self.declare_parameter(
            "action_name", "/navigate_to_pose"
        ).value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        self._base_frame = self.declare_parameter("base_frame", "base_link").value
        map_yaml = str(self.declare_parameter("map_yaml", "").value).strip()
        waypoint_file = str(self.declare_parameter("waypoint_file", "").value).strip()
        if waypoint_file:
            self._waypoint_file = Path(waypoint_file)
        else:
            if not map_yaml:
                raise ValueError("map_yaml is required when waypoint_file is empty")
            map_path = Path(map_yaml)
            self._waypoint_file = map_path.with_name(
                map_path.stem + "_waypoints.yaml"
            )
        self._occupied_threshold = int(
            self.declare_parameter("occupied_threshold", 50).value
        )
        self._clearance = self._positive_parameter("waypoint_clearance", 0.55)
        self._duplicate_distance = self._positive_parameter(
            "duplicate_distance", 0.20
        )
        self._maximum_waypoints = int(
            self.declare_parameter("maximum_waypoints", 100).value
        )
        self._arrival_pause = self._nonnegative_parameter("arrival_pause", 1.0)
        self._nomotion_service = str(
            self.declare_parameter(
                "nomotion_service", "/request_nomotion_update"
            ).value
        ).strip()
        self._nomotion_reobserve_delay = self._nonnegative_parameter(
            "nomotion_reobserve_delay", 1.0
        )
        self._localization_quality_topic = str(
            self.declare_parameter(
                "localization_quality_topic", "/localization/quality_ok"
            ).value
        ).strip()
        self._require_localization_quality = bool(
            self.declare_parameter("require_localization_quality", True).value
        )
        self._retry_delay = self._positive_parameter("retry_delay", 5.0)
        self._maximum_retries = int(
            self.declare_parameter("maximum_retries", 3).value
        )
        self._reverse_behavior_tree = str(
            self.declare_parameter("reverse_behavior_tree", "").value
        ).strip()
        self._reverse_max_distance = self._positive_parameter(
            "reverse_max_distance", 3.0
        )
        self._reverse_min_bearing = self._positive_parameter(
            "reverse_min_bearing", 2.0943951023931953
        )
        if not 1 <= self._occupied_threshold <= 100:
            raise ValueError("occupied_threshold must be in [1, 100]")
        if self._maximum_waypoints < 1 or self._maximum_retries < 0:
            raise ValueError("maximum_waypoints and maximum_retries are invalid")
        if self._reverse_min_bearing > math.pi:
            raise ValueError("reverse_min_bearing must not exceed pi radians")
        if not self._waypoint_file.is_absolute():
            raise ValueError("waypoint_file must be absolute")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, self._marker_topic, latched
        )
        self._active_publisher = self.create_publisher(
            Bool, self._active_topic, latched
        )
        self._status_publisher = self.create_publisher(
            String, self._status_topic, latched
        )
        self._localization_quality: Optional[bool] = None
        self.create_subscription(
            Bool,
            self._localization_quality_topic,
            self._on_localization_quality,
            latched,
        )
        self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, latched
        )
        self.create_subscription(
            PoseStamped, self._waypoint_topic, self._on_waypoint, 10
        )
        # Foxglove exposes one built-in 2D Point publisher even when a second
        # PoseStamped publisher cannot be added to the 3D panel. Accept both
        # schemas on the same operator-intent topic; DDS keeps them type-safe.
        self.create_subscription(
            PointStamped, self._waypoint_topic, self._on_point, 10
        )
        self.create_subscription(String, self._command_topic, self._on_command, 10)
        self.create_subscription(Empty, self._cancel_topic, self._on_cancel, 10)
        self._navigate_client = ActionClient(
            self, NavigateToPose, self._action_name
        )
        self._nomotion_client = self.create_client(
            EmptyService, self._nomotion_service
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._services = []
        for name, callback in {
            "start_once": lambda: self._start(False),
            "start_loop": lambda: self._start(True),
            "pause": self._pause,
            "resume": self._resume,
            "stop": self._stop,
            "undo": self._undo,
            "clear": self._clear,
            "reload": self._reload,
            "add_current_pose": self._add_current_pose,
        }.items():
            self._services.append(
                self.create_service(Trigger, "~/" + name, self._service(callback))
            )

        self._map: Optional[OccupancyGrid] = None
        self._waypoints: list[PoseStamped] = []
        self._auto_headings: list[bool] = []
        self._running = False
        self._paused = False
        self._loop = False
        self._current_index = 0
        self._retry_count = 0
        self._generation = 0
        self._goal_handle = None
        self._delay_timer = None
        self._last_message = "ready"

        self._load_file()
        self._publish_markers()
        self._publish_state("ready")
        self.get_logger().info(
            f"Waypoint patrol ready: add={self._waypoint_topic} "
            f"command={self._command_topic} file={self._waypoint_file} "
            f"count={len(self._waypoints)}"
        )

    def _positive_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _nonnegative_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and nonnegative")
        return value

    def _service(self, callback: Callable[[], tuple[bool, str]]):
        def handle(_request, response):
            response.success, response.message = callback()
            return response

        return handle

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _normalized_frame(self, frame: str) -> str:
        return frame[1:] if frame.startswith("/") else frame

    def _validate_pose(self, pose: PoseStamped) -> Optional[str]:
        if self._normalized_frame(pose.header.frame_id) != self._map_frame:
            return f"waypoint frame must be '{self._map_frame}'"
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return "waypoint contains a non-finite value"
        q = pose.pose.orientation
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm < 0.90 or norm > 1.10:
            return "waypoint quaternion is not normalized"
        if self._map is None:
            return "map has not been received"
        if not self._area_is_free(pose.pose.position.x, pose.pose.position.y):
            return "waypoint is unknown, occupied, outside the map, or lacks clearance"
        for existing in self._waypoints:
            dx = existing.pose.position.x - pose.pose.position.x
            dy = existing.pose.position.y - pose.pose.position.y
            if math.hypot(dx, dy) < self._duplicate_distance:
                return "waypoint is too close to an existing point"
        return None

    def _area_is_free(self, map_x: float, map_y: float) -> bool:
        assert self._map is not None
        info = self._map.info
        if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
            return False
        orientation = info.origin.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        yaw = math.atan2(sin_yaw, cos_yaw)
        dx = map_x - info.origin.position.x
        dy = map_y - info.origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        center_x = math.floor(local_x / info.resolution)
        center_y = math.floor(local_y / info.resolution)
        radius = math.ceil(self._clearance / info.resolution)
        for cell_y in range(center_y - radius, center_y + radius + 1):
            for cell_x in range(center_x - radius, center_x + radius + 1):
                if math.hypot(
                    (cell_x - center_x) * info.resolution,
                    (cell_y - center_y) * info.resolution,
                ) > self._clearance:
                    continue
                if (
                    cell_x < 0
                    or cell_y < 0
                    or cell_x >= info.width
                    or cell_y >= info.height
                ):
                    return False
                occupancy = self._map.data[cell_y * info.width + cell_x]
                if occupancy < 0 or occupancy >= self._occupied_threshold:
                    return False
        return True

    def _on_waypoint(self, message: PoseStamped) -> None:
        self._store_waypoint(message, auto_heading=False)

    def _on_point(self, message: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position = message.point
        pose.pose.orientation.w = 1.0
        self._store_waypoint(pose, auto_heading=True)

    def _store_waypoint(self, message: PoseStamped, auto_heading: bool) -> None:
        if self._running:
            self._publish_state("cannot edit waypoints while patrol is running")
            return
        if len(self._waypoints) >= self._maximum_waypoints:
            self._publish_state("maximum waypoint count reached")
            return
        failure = self._validate_pose(message)
        if failure:
            self.get_logger().warning(f"Rejected waypoint: {failure}")
            self._publish_state("rejected: " + failure)
            return
        stored = PoseStamped()
        stored.header.frame_id = self._map_frame
        stored.pose = message.pose
        self._waypoints.append(stored)
        self._auto_headings.append(auto_heading)
        self._save_file()
        self._publish_markers()
        heading = "automatic heading" if auto_heading else "operator heading"
        name = self._waypoint_name(len(self._waypoints) - 1)
        self._publish_state(f"added {name} ({heading})")

    def _on_command(self, message: String) -> None:
        command = message.data.strip().lower()
        actions = {
            "start_once": lambda: self._start(False),
            "start_loop": lambda: self._start(True),
            "pause": self._pause,
            "resume": self._resume,
            "stop": self._stop,
            "undo": self._undo,
            "clear": self._clear,
            "reload": self._reload,
            "add_current_pose": self._add_current_pose,
        }
        callback = actions.get(command)
        if callback is None:
            self._publish_state("unknown command: " + command)
            return
        success, text = callback()
        if not success:
            self.get_logger().warning(
                f"Patrol command {command} rejected: {text}"
            )

    def _on_cancel(self, _message: Empty) -> None:
        self._stop()

    def _on_localization_quality(self, message: Bool) -> None:
        self._localization_quality = bool(message.data)
        if (
            self._require_localization_quality
            and not self._localization_quality
            and self._running
        ):
            self._pause_for_localization(
                "laser scan no longer agrees with the static map"
            )

    def _pause_for_localization(self, reason: str) -> None:
        self._generation += 1
        self._running = False
        self._paused = True
        self._cancel_delay()
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._publish_state(
            "paused for localization: " + reason
            + "; correct initial pose, wait for /localization/quality_ok=true, then resume"
        )

    def _start(self, loop: bool) -> tuple[bool, str]:
        if self._running:
            return False, "patrol is already running"
        if not self._waypoints:
            return False, "no waypoints have been recorded"
        if self._require_localization_quality and self._localization_quality is not True:
            return False, (
                "localization quality is not ready; correct initial pose and wait for "
                f"{self._localization_quality_topic}=true"
            )
        if not self._navigate_client.wait_for_server(timeout_sec=0.5):
            return False, "NavigateToPose action server is unavailable"
        self._cancel_delay()
        self._generation += 1
        self._running = True
        self._paused = False
        self._loop = loop
        self._current_index = 0
        self._retry_count = 0
        message = "loop patrol started" if loop else "single patrol started"
        self._publish_state(message)
        self._send_current(self._generation)
        return True, message

    @staticmethod
    def _waypoint_name(index: int) -> str:
        """Return spreadsheet-style names: A..Z, AA..AZ, BA..."""
        value = index + 1
        name = ""
        while value:
            value, remainder = divmod(value - 1, 26)
            name = chr(ord("A") + remainder) + name
        return name

    def _add_current_pose(self) -> tuple[bool, str]:
        if self._running:
            return False, "cannot edit waypoints while patrol is running"
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as error:
            return False, f"cannot read current robot pose: {error}"
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        before = len(self._waypoints)
        self._store_waypoint(pose, auto_heading=False)
        if len(self._waypoints) == before:
            return False, self._last_message
        name = self._waypoint_name(len(self._waypoints) - 1)
        return True, f"recorded current robot pose as {name}"

    def _send_current(self, generation: int) -> None:
        if generation != self._generation or not self._running:
            return
        if self._require_localization_quality and self._localization_quality is not True:
            self._pause_for_localization("scan-to-map quality is below threshold")
            return
        pose = self._resolved_pose(self._current_index, self._loop)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose.pose
        reverse_description = self._reverse_approach_description(pose)
        if reverse_description is not None:
            goal.behavior_tree = self._reverse_behavior_tree
        self._publish_state(
            f"navigating to {self._waypoint_name(self._current_index)}, "
            f"attempt {self._retry_count + 1}"
            + (f", {reverse_description}" if reverse_description else "")
        )
        future = self._navigate_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, token=generation: self._on_goal_response(completed, token)
        )

    def _reverse_approach_description(self, target: PoseStamped) -> Optional[str]:
        """Select reverse navigation for a nearby target deep behind the body."""
        if not self._reverse_behavior_tree:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot evaluate reverse approach; using normal navigation: {error}"
            )
            return None

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx = target.pose.position.x - transform.transform.translation.x
        dy = target.pose.position.y - transform.transform.translation.y
        distance = math.hypot(dx, dy)
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        bearing = math.atan2(left, forward)
        if (
            distance > self._reverse_max_distance
            or abs(bearing) < self._reverse_min_bearing
        ):
            return None
        return (
            f"reverse approach distance={distance:.2f}m "
            f"bearing={math.degrees(bearing):.0f}deg"
        )

    def _on_goal_response(self, future, generation: int) -> None:
        if generation != self._generation or not self._running:
            return
        try:
            handle = future.result()
        except Exception as error:  # rclpy action transport failure
            self._on_goal_failure(generation, f"goal transport failed: {error}")
            return
        if not handle.accepted:
            self._on_goal_failure(generation, "NavigateToPose rejected the waypoint")
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, token=generation: self._on_goal_result(completed, token)
        )

    def _on_goal_result(self, future, generation: int) -> None:
        if generation != self._generation or not self._running:
            return
        self._goal_handle = None
        try:
            status = future.result().status
        except Exception as error:
            self._on_goal_failure(generation, f"result transport failed: {error}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._on_goal_failure(
                generation,
                f"{self._waypoint_name(self._current_index)} status={status}",
            )
            return
        self._retry_count = 0
        self._current_index += 1
        if self._current_index >= len(self._waypoints):
            if not self._loop:
                self._running = False
                self._publish_state("single patrol completed")
                return
            self._current_index = 0
        self._schedule(
            self._arrival_pause,
            lambda token=generation: self._request_nomotion_update(token),
        )

    def _request_nomotion_update(self, generation: int) -> None:
        if generation != self._generation or not self._running:
            return
        if not self._nomotion_service or not self._nomotion_client.service_is_ready():
            self.get_logger().warning(
                f"AMCL no-motion service {self._nomotion_service} is unavailable; "
                "continuing after the observation delay"
            )
            self._schedule(
                self._nomotion_reobserve_delay,
                lambda token=generation: self._send_current(token),
            )
            return
        future = self._nomotion_client.call_async(EmptyService.Request())
        future.add_done_callback(
            lambda completed, token=generation: self._on_nomotion_update(
                completed, token
            )
        )

    def _on_nomotion_update(self, future, generation: int) -> None:
        if generation != self._generation or not self._running:
            return
        try:
            future.result()
        except Exception as error:
            self.get_logger().warning(f"AMCL no-motion update failed: {error}")
        self._publish_state(
            "stationary localization refresh requested; validating before next leg"
        )
        self._schedule(
            self._nomotion_reobserve_delay,
            lambda token=generation: self._send_current(token),
        )

    def _on_goal_failure(self, generation: int, reason: str) -> None:
        if generation != self._generation or not self._running:
            return
        self._goal_handle = None
        if self._retry_count < self._maximum_retries:
            self._retry_count += 1
            self._publish_state(
                f"{reason}; retrying {self._waypoint_name(self._current_index)} after "
                f"{self._retry_delay:.1f}s"
            )
            self._schedule(
                self._retry_delay,
                lambda token=generation: self._send_current(token),
            )
            return
        self._running = False
        self._paused = True
        self._publish_state(
            f"paused at {self._waypoint_name(self._current_index)} after retries: {reason}"
        )

    def _pause(self) -> tuple[bool, str]:
        if not self._running:
            return False, "patrol is not running"
        self._generation += 1
        self._running = False
        self._paused = True
        self._cancel_delay()
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._publish_state(
            f"paused before {self._waypoint_name(self._current_index)}"
        )
        return True, "patrol paused"

    def _resume(self) -> tuple[bool, str]:
        if not self._paused:
            return False, "patrol is not paused"
        if not self._navigate_client.wait_for_server(timeout_sec=0.5):
            return False, "NavigateToPose action server is unavailable"
        if self._require_localization_quality and self._localization_quality is not True:
            return False, (
                "localization quality is still below threshold; correct initial pose and "
                f"wait for {self._localization_quality_topic}=true"
            )
        self._generation += 1
        self._running = True
        self._paused = False
        self._retry_count = 0
        self._publish_state(
            f"resuming at {self._waypoint_name(self._current_index)}"
        )
        self._send_current(self._generation)
        return True, "patrol resumed"

    def _stop(self) -> tuple[bool, str]:
        was_active = self._running or self._paused
        self._generation += 1
        self._running = False
        self._paused = False
        self._loop = False
        self._current_index = 0
        self._retry_count = 0
        self._cancel_delay()
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._publish_state("patrol stopped")
        return was_active, "patrol stopped" if was_active else "patrol was already stopped"

    def _undo(self) -> tuple[bool, str]:
        if self._running:
            return False, "cannot edit waypoints while patrol is running"
        if not self._waypoints:
            return False, "there are no waypoints to remove"
        removed = self._waypoint_name(len(self._waypoints) - 1)
        self._waypoints.pop()
        self._auto_headings.pop()
        self._current_index = min(self._current_index, max(0, len(self._waypoints) - 1))
        self._save_file()
        self._publish_markers()
        self._publish_state(f"removed {removed}")
        return True, f"removed {removed}"

    def _clear(self) -> tuple[bool, str]:
        if self._running:
            return False, "cannot edit waypoints while patrol is running"
        self._waypoints.clear()
        self._auto_headings.clear()
        self._current_index = 0
        self._paused = False
        self._save_file()
        self._publish_markers()
        self._publish_state("all waypoints cleared")
        return True, "all waypoints cleared"

    def _reload(self) -> tuple[bool, str]:
        if self._running:
            return False, "cannot reload waypoints while patrol is running"
        try:
            self._load_file()
        except Exception as error:
            self._publish_state(f"reload failed: {error}")
            return False, str(error)
        self._publish_markers()
        self._publish_state(f"reloaded {len(self._waypoints)} waypoints")
        return True, f"reloaded {len(self._waypoints)} waypoints"

    def _schedule(self, delay: float, callback: Callable[[], None]) -> None:
        self._cancel_delay()
        if delay <= 0.0:
            callback()
            return

        def fire() -> None:
            timer = self._delay_timer
            self._delay_timer = None
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            callback()

        self._delay_timer = self.create_timer(delay, fire)

    def _cancel_delay(self) -> None:
        if self._delay_timer is not None:
            self._delay_timer.cancel()
            self.destroy_timer(self._delay_timer)
            self._delay_timer = None

    def _save_file(self) -> None:
        payload = {
            "version": 1,
            "frame_id": self._map_frame,
            "waypoints": [
                {
                    "name": self._waypoint_name(index),
                    "x": float(pose.pose.position.x),
                    "y": float(pose.pose.position.y),
                    "z": float(pose.pose.position.z),
                    "qx": float(pose.pose.orientation.x),
                    "qy": float(pose.pose.orientation.y),
                    "qz": float(pose.pose.orientation.z),
                    "qw": float(pose.pose.orientation.w),
                    "auto_heading": self._auto_headings[index],
                }
                for index, pose in enumerate(self._waypoints)
            ],
        }
        self._waypoint_file.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=self._waypoint_file.name + ".",
            dir=str(self._waypoint_file.parent),
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(payload, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self._waypoint_file)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _load_file(self) -> None:
        self._waypoints = []
        self._auto_headings = []
        if not self._waypoint_file.exists():
            return
        with self._waypoint_file.open(encoding="utf-8") as stream:
            payload = yaml.safe_load(stream) or {}
        if payload.get("version") != 1 or payload.get("frame_id") != self._map_frame:
            raise ValueError("waypoint file version or frame_id is invalid")
        entries = payload.get("waypoints", [])
        if not isinstance(entries, list) or len(entries) > self._maximum_waypoints:
            raise ValueError("waypoint list is invalid or too large")
        for entry in entries:
            values = [
                float(entry[key])
                for key in ("x", "y", "z", "qx", "qy", "qz", "qw")
            ]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("waypoint file contains a non-finite value")
            norm = sum(value * value for value in values[3:])
            if norm < 0.90 or norm > 1.10:
                raise ValueError("waypoint file contains an invalid quaternion")
            pose = PoseStamped()
            pose.header.frame_id = self._map_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values[:3]
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = values[3:]
            self._waypoints.append(pose)
            self._auto_headings.append(bool(entry.get("auto_heading", False)))

    def _resolved_pose(self, index: int, loop: bool) -> PoseStamped:
        source = self._waypoints[index]
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = source.pose.position.x
        pose.pose.position.y = source.pose.position.y
        pose.pose.position.z = source.pose.position.z
        pose.pose.orientation.x = source.pose.orientation.x
        pose.pose.orientation.y = source.pose.orientation.y
        pose.pose.orientation.z = source.pose.orientation.z
        pose.pose.orientation.w = source.pose.orientation.w
        if not self._auto_headings[index] or len(self._waypoints) < 2:
            return pose

        if index + 1 < len(self._waypoints):
            start = self._waypoints[index].pose.position
            end = self._waypoints[index + 1].pose.position
        elif loop:
            start = self._waypoints[index].pose.position
            end = self._waypoints[0].pose.position
        else:
            start = self._waypoints[index - 1].pose.position
            end = self._waypoints[index].pose.position
        yaw = math.atan2(end.y - start.y, end.x - start.x)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        line = Marker()
        line.header.frame_id = self._map_frame
        line.header.stamp = now
        line.ns = "patrol_route"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.035
        line.color.r = 0.10
        line.color.g = 0.75
        line.color.b = 1.0
        line.color.a = 0.75

        for index in range(len(self._waypoints)):
            pose = self._resolved_pose(index, self._loop)
            arrow = Marker()
            arrow.header.frame_id = self._map_frame
            arrow.header.stamp = now
            arrow.ns = "patrol_waypoints"
            arrow.id = 2 * index + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose.pose
            arrow.pose.position.z = 0.08
            arrow.scale.x = 0.45
            arrow.scale.y = 0.10
            arrow.scale.z = 0.10
            arrow.color.r = 0.10
            arrow.color.g = 0.90
            arrow.color.b = 0.35
            arrow.color.a = 1.0
            markers.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = "patrol_labels"
            label.id = 2 * index + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pose.pose.position.x
            label.pose.position.y = pose.pose.position.y
            label.pose.position.z = 0.42
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = self._waypoint_name(index)
            markers.markers.append(label)

            route_point = Point()
            route_point.x = pose.pose.position.x
            route_point.y = pose.pose.position.y
            route_point.z = 0.05
            line.points.append(route_point)
        if line.points:
            markers.markers.append(line)
        self._marker_publisher.publish(markers)

    def _publish_state(self, message: str) -> None:
        self._last_message = message
        active = Bool()
        active.data = self._running
        self._active_publisher.publish(active)
        status = String()
        status.data = json.dumps(
            {
                "state": "running" if self._running else ("paused" if self._paused else "idle"),
                "loop": self._loop,
                "waypoint_count": len(self._waypoints),
                "current_waypoint": self._current_index + 1 if self._waypoints else 0,
                "current_waypoint_name": (
                    self._waypoint_name(self._current_index)
                    if self._waypoints else ""
                ),
                "retry": self._retry_count,
                "localization_quality_ok": self._localization_quality,
                "message": message,
                "file": str(self._waypoint_file),
            },
            ensure_ascii=False,
        )
        self._status_publisher.publish(status)
        self.get_logger().info(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointPatrolNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
