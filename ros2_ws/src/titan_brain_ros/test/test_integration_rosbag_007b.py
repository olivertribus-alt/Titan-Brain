"""Synthetic replay integration test for the TB-EVAL-007B ROS control plane."""

from __future__ import annotations

import math
import time
import unittest
from collections.abc import Callable
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from titan_brain_msgs.msg import (
    ArbitrationStatus,
    SystemFaultStatus,
)

_PACKAGE_NAME = "titan_brain_ros"
_DISCOVERY_TIMEOUT_SEC = 30.0
_SCENARIO_TIMEOUT_SEC = 5.0


def _reliable_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    """Launch the opt-in 007B profile used for deployment."""
    package_share = Path(get_package_share_directory(_PACKAGE_NAME))
    launch_file = package_share / "launch" / "safety_control_plane.launch.py"
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSafetyVelocityArbiterReplay(unittest.TestCase):
    """Replay deterministic scan, command, and fault frames over real DDS."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = Node("tb_eval_007b_replay_driver")
        cls.outputs: list[TwistStamped] = []
        cls.statuses: list[ArbitrationStatus] = []
        cls.teleop = cls.node.create_publisher(
            TwistStamped,
            "/teleop/cmd_vel",
            _reliable_qos(1),
        )
        cls.autonomy = cls.node.create_publisher(
            TwistStamped,
            "/autonomy/cmd_vel",
            _reliable_qos(1),
        )
        cls.fault = cls.node.create_publisher(
            SystemFaultStatus,
            "/safety/system_fault_status",
            _reliable_qos(10),
        )
        cls.scan = cls.node.create_publisher(
            LaserScan,
            "/scan",
            _sensor_qos(),
        )
        cls.output_subscription = cls.node.create_subscription(
            TwistStamped,
            "/cmd_vel",
            cls.outputs.append,
            _reliable_qos(1),
        )
        cls.status_subscription = cls.node.create_subscription(
            ArbitrationStatus,
            "/safety/arbitration_status",
            cls.statuses.append,
            _reliable_qos(10),
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _spin_until(
        self,
        predicate: Callable[[], bool],
        timeout_sec: float,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail("Timed out waiting for the replayed ROS 2 state")

    def _wait_for_graph(self) -> None:
        self._spin_until(
            lambda: (
                self.node.count_subscribers("/teleop/cmd_vel") == 2
                and self.node.count_subscribers("/autonomy/cmd_vel") == 2
                and self.node.count_subscribers("/safety/system_fault_status") == 4
                and self.node.count_subscribers("/scan") == 1
                and self.node.count_publishers("/safety/permitted_motion_envelope") == 1
                and self.node.count_publishers("/safety/lifecycle_status") == 1
                and self.node.count_publishers("/cmd_vel") == 1
            ),
            _DISCOVERY_TIMEOUT_SEC,
        )

    def _stamp_now(
        self,
        message: TwistStamped | LaserScan | SystemFaultStatus,
    ) -> int:
        now_ns = int(self.node.get_clock().now().nanoseconds)
        header = message.header
        header.stamp.sec = now_ns // 1_000_000_000
        header.stamp.nanosec = now_ns % 1_000_000_000
        return now_ns

    def _publish_replay_frame(self, fault_state: int) -> None:
        fault = SystemFaultStatus()
        self._stamp_now(fault)
        fault.fault_state = fault_state

        scan = LaserScan()
        scan.header.frame_id = "laser"
        self._stamp_now(scan)
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / 360
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0] * 360
        for index in range(135, 226):
            scan.ranges[index] = 0.35

        autonomy = TwistStamped()
        autonomy.header.frame_id = "base_link"
        self._stamp_now(autonomy)
        autonomy.twist.linear.x = 0.1

        teleop = TwistStamped()
        teleop.header.frame_id = "base_link"
        self._stamp_now(teleop)
        teleop.twist.linear.x = 0.8

        self.fault.publish(fault)
        self.scan.publish(scan)
        self.autonomy.publish(autonomy)
        self.teleop.publish(teleop)

    def test_synthetic_replay_priority_clamp_and_fault_stop(self) -> None:
        self._wait_for_graph()
        self.outputs.clear()
        self.statuses.clear()

        deadline = time.monotonic() + _SCENARIO_TIMEOUT_SEC
        while time.monotonic() < deadline:
            self._publish_replay_frame(SystemFaultStatus.FAULT_OK)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if any(
                status.active_source == "teleoperation"
                and status.mode == status.MODE_CLAMPED
                for status in self.statuses
            ):
                break
        else:
            self.fail("Teleoperation replay was not selected and clamped")

        self.assertTrue(self.outputs)
        self.assertLessEqual(abs(self.outputs[-1].twist.linear.x), 0.30)

        self.outputs.clear()
        self.statuses.clear()
        deadline = time.monotonic() + _SCENARIO_TIMEOUT_SEC
        while time.monotonic() < deadline:
            self._publish_replay_frame(SystemFaultStatus.FAULT_E_STOP_ACTIVE)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if any(
                status.rejection_reason == "SYSTEM_FAULT_E_STOP_ACTIVE"
                for status in self.statuses
            ):
                break
        else:
            self.fail("E-stop replay did not reach the output authority")

        self.assertTrue(self.outputs)
        self.assertEqual(self.outputs[-1].twist.linear.x, 0.0)
        self.assertEqual(self.outputs[-1].twist.angular.z, 0.0)


@launch_testing.post_shutdown_test()
class TestSafetyVelocityArbiterShutdown(unittest.TestCase):
    """Assert that the 007B launch exits without process failures."""

    def test_exit_codes(self, proc_info: object) -> None:
        launch_testing.asserts.assertExitCodes(proc_info)
