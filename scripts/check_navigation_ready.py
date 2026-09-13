#!/usr/bin/env python3
"""Read-only operator preflight for the guarded first-navigation workflow."""

import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class NavigationPreflight(Node):
    def __init__(self) -> None:
        super().__init__("navigation_preflight_once")
        self.map_seen = False
        self.pose_seen = False
        self.cloud_seen = False
        self.statuses = {}

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map, latched)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose, reliable
        )
        self.create_subscription(PointCloud2, "/navigation/obstacles", self._cloud, sensor)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics, reliable)

    def _map(self, _message: OccupancyGrid) -> None:
        self.map_seen = True

    def _pose(self, _message: PoseWithCovarianceStamped) -> None:
        self.pose_seen = True

    def _cloud(self, _message: PointCloud2) -> None:
        self.cloud_seen = True

    def _diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name in {
                "u_robot/navigation_cloud",
                "u_robot/operator_goal_gate",
            }:
                self.statuses[status.name] = status


def status_values(status: DiagnosticStatus):
    return {item.key: item.value for item in status.values}


def main() -> int:
    rclpy.init()
    node = NavigationPreflight()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (
            node.map_seen
            and node.cloud_seen
            and len(node.statuses) == 2
        ):
            break

    cloud = node.statuses.get("u_robot/navigation_cloud")
    gate = node.statuses.get("u_robot/operator_goal_gate")
    cloud_values = status_values(cloud) if cloud else {}
    gate_values = status_values(gate) if gate else {}
    # AMCL publishes on filter updates rather than as a periodic heartbeat. The
    # long-running goal gate retains whether a valid pose has been received.
    pose_ready = node.pose_seen or gate_values.get("pose_received") == "true"
    checks = [
        (node.map_seen, "静态地图 /map 已加载"),
        (pose_ready, "AMCL 已收到并保留有效初始定位"),
        (node.cloud_seen, "导航障碍点云 /navigation/obstacles 正常刷新"),
    ]
    checks.extend(
        [
            (
                cloud is not None and cloud.level == DiagnosticStatus.OK,
                "点云过滤诊断正常"
                if cloud is not None and cloud.level == DiagnosticStatus.OK
                else f"点云过滤未就绪：{cloud.message if cloud else '没有诊断'}",
            ),
            (
                cloud_values.get("dynamic_self_mask_ready") == "true",
                "URDF/TF 动态自体遮罩已就绪"
                if cloud_values.get("dynamic_self_mask_ready") == "true"
                else "动态自体遮罩未就绪（若字段缺失，请重启导航加载新版本）",
            ),
            (
                gate is not None and gate.level == DiagnosticStatus.OK,
                "目标门禁允许接收 /operator/goal_pose"
                if gate is not None and gate.level == DiagnosticStatus.OK
                else f"目标门禁未就绪：{gate.message if gate else '没有诊断'}",
            ),
        ]
    )

    success = True
    for passed, description in checks:
        success = success and passed
        print(f"[{'PASS' if passed else 'FAIL'}] {description}")
    if success:
        print("\nREADY：现在可以在 Foxglove 3D 面板使用 2D pose 工具下发 dry-run 目标。")
    else:
        print("\nNOT READY：不要下发目标；先处理上面的 FAIL 项。")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
