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
    DirectionalSafetyObservation,
    SafetyEvaluationStatus,
    SafetyObservation,
)

_PACKAGE_NAME = "titan_brain_ros"
_DISCOVERY_TIMEOUT_SEC = 10.0
_SCENARIO_TIMEOUT_SEC = 5.0
_DRIVER_POLL_PERIOD_SEC = 0.005
_INPUT_STREAM_PERIOD_NS = 20_000_000
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
            DirectionalSafetyObservation,
            "/safety/directional_observation",
            _sensor_qos(),
        )
        cls.legacy_observation_publisher = cls.node.create_publisher(
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
        failure_message: str | Callable[[], str],
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
        detail = failure_message() if callable(failure_message) else failure_message
        self.fail(detail)

    def _publish_observation(
        self,
        *,
        clearance_m: float,
        linear_x_mps: float = 0.4,
        reverse_clearance_m: float = 10.0,
        left_clearance_m: float = 10.0,
        right_clearance_m: float = 10.0,
    ) -> None:
        message = DirectionalSafetyObservation()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.map_id = "e2e_test_map"
        message.pose.x = 1.0
        message.pose.y = 2.0
        message.pose.theta = 0.0
        message.clearance_m = clearance_m
        message.confidence = 0.95
        message.sensor_id = "e2e_front_lidar"
        message.forward_clearance_m = clearance_m
        message.reverse_clearance_m = reverse_clearance_m
        message.left_clearance_m = left_clearance_m
        message.right_clearance_m = right_clearance_m
        message.velocity.linear.x = linear_x_mps
        message.velocity.linear.y = 0.0
        message.velocity.angular.z = 0.0
        self.observation_publisher.publish(message)

    def _publish_navigation_command(self, *, linear_x_mps: float = 0.4) -> None:
        message = Twist()
        message.linear.x = linear_x_mps
        message.linear.y = 0.0
        message.angular.z = 0.0
        self.navigation_publisher.publish(message)

    def _publish_legacy_observation(self, *, clearance_m: float) -> None:
        message = SafetyObservation()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.map_id = "e2e_test_map"
        message.pose.x = 1.0
        message.pose.y = 2.0
        message.pose.theta = 0.0
        message.clearance_m = clearance_m
        message.confidence = 0.95
        message.sensor_id = "legacy_front_lidar"
        self.legacy_observation_publisher.publish(message)

    def _periodic_stream(
        self,
        publish: Callable[[], None],
    ) -> Callable[[], None]:
        """Return a non-blocking 50 Hz publisher callback."""
        next_publish_at_ns = 0

        def publish_if_due() -> None:
            nonlocal next_publish_at_ns
            now_ns = time.monotonic_ns()
            if now_ns < next_publish_at_ns:
                return
            publish()
            next_publish_at_ns = now_ns + _INPUT_STREAM_PERIOD_NS

        return publish_if_due

    def _transport_diagnostics(self) -> str:
        evaluation_states = [
            (
                message.adapter_status,
                message.watchdog_status,
                message.observation_accepted,
                message.action,
            )
            for message in self.evaluation_events[-10:]
        ]
        arbitration_reasons = [
            message.reason for message, _received_at_ns in self.arbitration_events[-20:]
        ]
        return (
            f"recent evaluations={evaluation_states!r}; "
            f"recent arbitration reasons={arbitration_reasons!r}"
        )

    def _stream_inputs_until_arbitration(
        self,
        *,
        clearance_m: float,
        evaluation_action: str,
        arbitration_reason: str,
        linear_x_mps: float = 0.4,
        reverse_clearance_m: float = 10.0,
        left_clearance_m: float = 10.0,
        right_clearance_m: float = 10.0,
    ) -> int:
        """Keep both independent inputs fresh until their result is observed."""
        first_observation_at_ns: int | None = None

        def publish_observation() -> None:
            nonlocal first_observation_at_ns
            if first_observation_at_ns is None:
                first_observation_at_ns = time.monotonic_ns()
            self._publish_observation(
                clearance_m=clearance_m,
                linear_x_mps=linear_x_mps,
                reverse_clearance_m=reverse_clearance_m,
                left_clearance_m=left_clearance_m,
                right_clearance_m=right_clearance_m,
            )

        observation_stream = self._periodic_stream(publish_observation)
        navigation_stream = self._periodic_stream(
            lambda: self._publish_navigation_command(
                linear_x_mps=linear_x_mps
            )
        )

        def keep_inputs_fresh() -> None:
            observation_stream()
            navigation_stream()

        self._wait_for(
            lambda: (
                any(
                    message.observation_accepted
                    and message.action == evaluation_action
                    for message in self.evaluation_events
                )
                and any(
                    message.reason == arbitration_reason
                    for message, _received_at_ns in self.arbitration_events
                )
            ),
            on_cycle=keep_inputs_fresh,
            failure_message=lambda: (
                "No synchronized "
                f"{evaluation_action!r}/{arbitration_reason!r} result was "
                f"received; {self._transport_diagnostics()}"
            ),
        )
        assert first_observation_at_ns is not None
        return first_observation_at_ns

    def _wait_for_arbitration_reason(
        self,
        reason: str,
        *,
        on_cycle: Callable[[], None] | None = None,
    ) -> None:
        self._wait_for(
            lambda: any(
                message.reason == reason
                for message, _received_at_ns in self.arbitration_events
            ),
            on_cycle=on_cycle,
            failure_message=lambda: (
                f"No {reason!r} arbitration result was received; "
                f"{self._transport_diagnostics()}"
            ),
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
                and self.legacy_observation_publisher.get_subscription_count()
                == 1
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
            self._stream_inputs_until_arbitration(
                clearance_m=1.2,
                evaluation_action="proceed",
                arbitration_reason="proceed",
            )
            self._wait_for(
                lambda: any(
                    message.linear.x == 0.4
                    and message.linear.y == 0.0
                    and message.angular.z == 0.0
                    for message, _received_at_ns in self.cmd_vel_events
                ),
                failure_message="Pass-through command was not published.",
            )

        with self.subTest("speed_expands_forward_braking_envelope"):
            self._clear_events()
            self._stream_inputs_until_arbitration(
                clearance_m=0.8,
                linear_x_mps=0.1,
                evaluation_action="proceed",
                arbitration_reason="proceed",
            )
            self._clear_events()
            self._stream_inputs_until_arbitration(
                clearance_m=0.8,
                linear_x_mps=0.8,
                evaluation_action="emergency_stop",
                arbitration_reason="emergency_stop",
            )

        with self.subTest("legacy_input_fails_closed_in_dynamic_mode"):
            self._clear_events()
            legacy_stream = self._periodic_stream(
                lambda: self._publish_legacy_observation(clearance_m=1.2)
            )
            navigation_stream = self._periodic_stream(
                self._publish_navigation_command
            )

            def keep_legacy_inputs_fresh() -> None:
                legacy_stream()
                navigation_stream()

            self._wait_for(
                lambda: (
                    any(
                        message.observation_accepted
                        and message.rule == "EV-SAFE-DYN-03"
                        and message.action == "protective_stop"
                        for message in self.evaluation_events
                    )
                    and any(
                        message.reason == "protective_stop"
                        for message, _received_at_ns in self.arbitration_events
                    )
                ),
                on_cycle=keep_legacy_inputs_fresh,
                failure_message=lambda: (
                    "Legacy observation did not fail closed in dynamic mode; "
                    f"{self._transport_diagnostics()}"
                ),
            )

        with self.subTest("inactive_sectors_do_not_block_forward_motion"):
            self._clear_events()
            self._stream_inputs_until_arbitration(
                clearance_m=1.2,
                reverse_clearance_m=0.01,
                left_clearance_m=0.01,
                right_clearance_m=0.01,
                evaluation_action="proceed",
                arbitration_reason="proceed",
            )

        with self.subTest("emergency_stop_meets_ci_deadline"):
            self._clear_events()
            published_at_ns = self._stream_inputs_until_arbitration(
                clearance_m=0.2,
                evaluation_action="emergency_stop",
                arbitration_reason="emergency_stop",
            )
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
            self._stream_inputs_until_arbitration(
                clearance_m=1.2,
                evaluation_action="proceed",
                arbitration_reason="proceed",
            )
            self.arbitration_events.clear()
            self.cmd_vel_events.clear()
            observation_stream = self._periodic_stream(
                lambda: self._publish_observation(clearance_m=1.2)
            )
            self._wait_for_arbitration_reason(
                "command_stale",
                on_cycle=observation_stream,
            )
            stale_status = next(
                message
                for message, _received_at_ns in self.arbitration_events
                if message.reason == "command_stale"
            )
            self.assertEqual(stale_status.commanded_twist.linear.x, 0.0)

        with self.subTest("observation_watchdog_times_out"):
            self._clear_events()
            self._stream_inputs_until_arbitration(
                clearance_m=1.2,
                evaluation_action="proceed",
                arbitration_reason="proceed",
            )
            self.arbitration_events.clear()
            self.cmd_vel_events.clear()
            navigation_stream = self._periodic_stream(
                self._publish_navigation_command
            )
            self._wait_for_arbitration_reason(
                "watchdog_timed_out",
                on_cycle=navigation_stream,
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
