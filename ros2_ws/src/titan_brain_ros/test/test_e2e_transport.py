"""Live DDS integration test for the complete Titan Brain ROS 2 pipeline."""

from __future__ import annotations

import threading
import time
import unittest
from collections.abc import Callable
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from titan_brain_msgs.msg import (
    ArbitrationStatus,
    SafetyEvaluationStatus,
    SafetyObservation,
)

_PACKAGE_NAME = "titan_brain_ros"
_DISCOVERY_TIMEOUT_SEC = 10.0
_SCENARIO_TIMEOUT_SEC = 5.0
_DRIVER_POLL_PERIOD_SEC = 0.005
_OBSERVATION_PERIOD_NS = 20_000_000
_CI_STOP_DEADLINE_NS = 250_000_000


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    """Launch the installed production description for active DDS testing."""
    package_share = Path(get_package_share_directory(_PACKAGE_NAME))
    launch_file = package_share / "launch" / "titan_brain.launch.py"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file))
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _command_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _status_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class TestTitanBrainTransport(unittest.TestCase):
    """Exercise both launched nodes through actual ROS publishers/subscribers."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = Node("titan_brain_e2e_test_driver")
        cls.cmd_vel_events: list[tuple[Twist, int]] = []
        cls.arbitration_events: list[tuple[ArbitrationStatus, int]] = []
        cls.evaluation_events: list[SafetyEvaluationStatus] = []

        cls.observation_publisher = cls.node.create_publisher(
            SafetyObservation,
            "/safety/observation",
            _sensor_qos(),
        )
        cls.navigation_publisher = cls.node.create_publisher(
            Twist,
            "/cmd_vel_nav",
            _command_qos(),
        )
        cls.cmd_vel_subscription = cls.node.create_subscription(
            Twist,
            "/cmd_vel",
            cls._on_cmd_vel,
            _command_qos(),
        )
        cls.arbitration_subscription = cls.node.create_subscription(
            ArbitrationStatus,
            "/safety/arbitration_status",
            cls._on_arbitration,
            _status_qos(),
        )
        cls.evaluation_subscription = cls.node.create_subscription(
            SafetyEvaluationStatus,
            "/safety/evaluation_status",
            cls._on_evaluation,
            _status_qos(),
        )
        cls.executor = SingleThreadedExecutor(context=cls.node.context)
        cls.executor.add_node(cls.node)
        cls.executor_thread = threading.Thread(
            target=cls.executor.spin,
            name="titan-brain-e2e-executor",
            daemon=True,
        )
        cls.executor_thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.executor.shutdown(timeout_sec=2.0)
        cls.executor_thread.join(timeout=2.0)
        if cls.executor_thread.is_alive():
            raise RuntimeError("E2E test executor did not shut down cleanly")
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_cmd_vel(cls, message: Twist) -> None:
        cls.cmd_vel_events.append((message, time.monotonic_ns()))

    @classmethod
    def _on_arbitration(cls, message: ArbitrationStatus) -> None:
        cls.arbitration_events.append((message, time.monotonic_ns()))

    @classmethod
    def _on_evaluation(cls, message: SafetyEvaluationStatus) -> None:
        cls.evaluation_events.append(message)

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        *,
        timeout_sec: float = _SCENARIO_TIMEOUT_SEC,
        on_cycle: Callable[[], None] | None = None,
        failure_message: str,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if on_cycle is not None:
                on_cycle()
            if predicate():
                return
            if not self.executor_thread.is_alive():
                self.fail("E2E test executor stopped unexpectedly.")
            time.sleep(_DRIVER_POLL_PERIOD_SEC)
        if predicate():
            return
        self.fail(failure_message)

    def _publish_observation(self, *, clearance_m: float) -> None:
        message = SafetyObservation()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.map_id = "e2e_test_map"
        message.pose.x = 1.0
        message.pose.y = 2.0
        message.pose.theta = 0.0
        message.clearance_m = clearance_m
        message.confidence = 0.95
        message.sensor_id = "e2e_front_lidar"
        self.observation_publisher.publish(message)

    def _publish_navigation_command(self) -> None:
        message = Twist()
        message.linear.x = 0.4
        message.linear.y = 0.1
        message.angular.z = 0.5
        self.navigation_publisher.publish(message)

    def _stream_until_evaluation(
        self,
        action: str,
        *,
        clearance_m: float,
    ) -> int:
        """Stream best-effort samples and return the first publication time."""
        first_published_at_ns: int | None = None
        next_publish_at_ns = 0

        def publish_sensor_sample() -> None:
            nonlocal first_published_at_ns, next_publish_at_ns
            now_ns = time.monotonic_ns()
            if now_ns < next_publish_at_ns:
                return
            if first_published_at_ns is None:
                first_published_at_ns = now_ns
            self._publish_observation(clearance_m=clearance_m)
            next_publish_at_ns = now_ns + _OBSERVATION_PERIOD_NS

        self._wait_for(
            lambda: any(
                message.observation_accepted and message.action == action
                for message in self.evaluation_events
            ),
            on_cycle=publish_sensor_sample,
            failure_message=f"No accepted {action!r} evaluation was received.",
        )
        assert first_published_at_ns is not None
        return first_published_at_ns

    def _wait_for_arbitration_reason(self, reason: str) -> None:
        self._wait_for(
            lambda: any(
                message.reason == reason
                for message, _received_at_ns in self.arbitration_events
            ),
            failure_message=f"No {reason!r} arbitration result was received.",
        )

    def _clear_events(self) -> None:
        self.cmd_vel_events.clear()
        self.arbitration_events.clear()
        self.evaluation_events.clear()

    def test_complete_safety_transport_pipeline(self) -> None:
        """Verify startup, motion, stop, and staleness over live DDS topics."""
        self._wait_for(
            lambda: (
                self.observation_publisher.get_subscription_count() == 1
                and self.navigation_publisher.get_subscription_count() == 1
                and self.node.count_publishers("/cmd_vel") == 1
                and self.node.count_publishers("/safety/arbitration_status") == 1
                and self.node.count_publishers("/safety/evaluation_status") == 1
            ),
            timeout_sec=_DISCOVERY_TIMEOUT_SEC,
            failure_message="Expected Titan Brain ROS graph was not discovered.",
        )

        with self.subTest("cold_start_forces_zero"):
            self._clear_events()
            self._wait_for_arbitration_reason("safety_state_missing")
            startup_status = next(
                message
                for message, _received_at_ns in self.arbitration_events
                if message.reason == "safety_state_missing"
            )
            self.assertEqual(
                startup_status.mode,
                ArbitrationStatus.MODE_FORCED_ZERO,
            )
            self.assertEqual(startup_status.commanded_twist.linear.x, 0.0)

        with self.subTest("safe_observation_passes_command"):
            self._clear_events()
            self._stream_until_evaluation("proceed", clearance_m=1.2)
            self._publish_navigation_command()
            self._wait_for_arbitration_reason("proceed")
            self._wait_for(
                lambda: any(
                    message.linear.x == 0.4
                    and message.linear.y == 0.1
                    and message.angular.z == 0.5
                    for message, _received_at_ns in self.cmd_vel_events
                ),
                failure_message="Pass-through command was not published.",
            )

        with self.subTest("emergency_stop_meets_ci_deadline"):
            self._clear_events()
            published_at_ns = self._stream_until_evaluation(
                "emergency_stop",
                clearance_m=0.2,
            )
            self._wait_for_arbitration_reason("emergency_stop")
            stop_status, received_at_ns = next(
                event
                for event in self.arbitration_events
                if event[0].reason == "emergency_stop"
            )
            measured_latency_ns = received_at_ns - published_at_ns
            self.assertEqual(
                stop_status.mode,
                ArbitrationStatus.MODE_FORCED_ZERO,
            )
            self.assertEqual(stop_status.commanded_twist.linear.x, 0.0)
            self.assertLess(
                measured_latency_ns,
                _CI_STOP_DEADLINE_NS,
                "DDS emergency-stop reaction exceeded the non-RT CI deadline: "
                f"{measured_latency_ns / 1_000_000:.3f} ms",
            )
            self._wait_for(
                lambda: any(
                    message.linear.x == 0.0
                    and received_ns >= published_at_ns
                    for message, received_ns in self.cmd_vel_events
                ),
                failure_message="Emergency zero was not observed on /cmd_vel.",
            )

        with self.subTest("navigation_command_becomes_stale"):
            self._clear_events()
            self._stream_until_evaluation("proceed", clearance_m=1.2)
            self._publish_navigation_command()
            self._wait_for_arbitration_reason("proceed")
            self.arbitration_events.clear()
            self.cmd_vel_events.clear()
            self._wait_for_arbitration_reason("command_stale")
            stale_status = next(
                message
                for message, _received_at_ns in self.arbitration_events
                if message.reason == "command_stale"
            )
            self.assertEqual(stale_status.commanded_twist.linear.x, 0.0)

        with self.subTest("observation_watchdog_times_out"):
            self._clear_events()
            self._stream_until_evaluation("proceed", clearance_m=1.2)
            self._publish_navigation_command()
            self._wait_for_arbitration_reason("proceed")
            self.arbitration_events.clear()
            self.cmd_vel_events.clear()

            next_command_at = time.monotonic()

            def keep_navigation_fresh() -> None:
                nonlocal next_command_at
                now = time.monotonic()
                if now >= next_command_at:
                    self._publish_navigation_command()
                    next_command_at = now + 0.03

            self._wait_for(
                lambda: any(
                    message.reason == "watchdog_timed_out"
                    for message, _received_at_ns in self.arbitration_events
                ),
                on_cycle=keep_navigation_fresh,
                failure_message="Observation watchdog did not force zero.",
            )
            timeout_status = next(
                message
                for message, _received_at_ns in self.arbitration_events
                if message.reason == "watchdog_timed_out"
            )
            self.assertEqual(
                timeout_status.mode,
                ArbitrationStatus.MODE_FORCED_ZERO,
            )
            self.assertEqual(timeout_status.commanded_twist.linear.x, 0.0)


@launch_testing.post_shutdown_test()
class TestTitanBrainProcessExit(unittest.TestCase):
    """Verify that both launched node processes shut down cleanly."""

    def test_exit_codes(self, proc_info: launch_testing.ProcInfoHandler) -> None:
        launch_testing.asserts.assertExitCodes(proc_info)
