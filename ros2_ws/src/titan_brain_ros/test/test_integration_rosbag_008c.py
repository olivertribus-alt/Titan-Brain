"""Live DDS fault-injection gate for the TB-EVAL-008 safety chain."""

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
    EnvelopeDiagnostics,
    PermittedMotionEnvelope,
    SystemFaultStatus,
)

_PACKAGE_NAME = "titan_brain_ros"
_DISCOVERY_TIMEOUT_SEC = 30.0
_SCENARIO_TIMEOUT_SEC = 5.0
_SENSOR_TIMEOUT_SEC = 0.20
_INPUT_STREAM_PERIOD_NS = 20_000_000
_ENVELOPE_REACTION_BUDGET_NS = 100_000_000
_CHAIN_STOP_BUDGET_NS = 250_000_000


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


def _stamp_ns(
    message: (
        ArbitrationStatus
        | EnvelopeDiagnostics
        | LaserScan
        | PermittedMotionEnvelope
        | SystemFaultStatus
        | TwistStamped
    ),
) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    """Launch the installed 008 safety control plane under active injection."""
    package_share = Path(get_package_share_directory(_PACKAGE_NAME))
    launch_file = package_share / "launch" / "safety_control_plane.launch.py"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file))
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDynamicEnvelopeFaultInjection(unittest.TestCase):
    """Inject sensor and control-plane faults over real ROS 2 DDS topics."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = Node("tb_eval_008c_fault_injection_driver")
        cls.envelopes: list[PermittedMotionEnvelope] = []
        cls.diagnostics: list[EnvelopeDiagnostics] = []
        cls.outputs: list[TwistStamped] = []
        cls.statuses: list[ArbitrationStatus] = []
        cls.last_scan_timestamp_ns = 0

        cls.teleop = cls.node.create_publisher(
            TwistStamped,
            "/teleop/cmd_vel",
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
        cls.envelope_subscription = cls.node.create_subscription(
            PermittedMotionEnvelope,
            "/safety/permitted_motion_envelope",
            cls.envelopes.append,
            _reliable_qos(10),
        )
        cls.diagnostics_subscription = cls.node.create_subscription(
            EnvelopeDiagnostics,
            "/safety/envelope_diagnostics",
            cls.diagnostics.append,
            _reliable_qos(10),
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
        *,
        timeout_sec: float = _SCENARIO_TIMEOUT_SEC,
        on_cycle: Callable[[], None] | None = None,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if on_cycle is not None:
                on_cycle()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail("Timed out waiting for the injected ROS 2 safety state")

    def _wait_for_graph(self) -> None:
        self._spin_until(
            lambda: (
                self.node.count_subscribers("/teleop/cmd_vel") == 1
                and self.node.count_subscribers(
                    "/safety/system_fault_status"
                )
                == 2
                and self.node.count_subscribers("/scan") == 1
                and self.node.count_publishers(
                    "/safety/permitted_motion_envelope"
                )
                == 1
                and self.node.count_publishers(
                    "/safety/envelope_diagnostics"
                )
                == 1
                and self.node.count_publishers("/cmd_vel") == 1
            ),
            timeout_sec=_DISCOVERY_TIMEOUT_SEC,
        )

    def _periodic_stream(
        self,
        publish: Callable[[], None],
    ) -> Callable[[], None]:
        next_publish_at_ns = 0

        def publish_if_due() -> None:
            nonlocal next_publish_at_ns
            now_ns = time.monotonic_ns()
            if now_ns < next_publish_at_ns:
                return
            publish()
            next_publish_at_ns = now_ns + _INPUT_STREAM_PERIOD_NS

        return publish_if_due

    def _next_scan_timestamp_ns(self) -> int:
        now_ns = int(self.node.get_clock().now().nanoseconds)
        timestamp_ns = max(now_ns, self.last_scan_timestamp_ns + 1)
        self.last_scan_timestamp_ns = timestamp_ns
        return timestamp_ns

    def _set_stamp(
        self,
        message: LaserScan | SystemFaultStatus | TwistStamped,
        timestamp_ns: int,
    ) -> None:
        message.header.stamp.sec = timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = timestamp_ns % 1_000_000_000

    def _publish_control(
        self,
        fault_state: int = SystemFaultStatus.FAULT_OK,
    ) -> None:
        now_ns = int(self.node.get_clock().now().nanoseconds)

        fault = SystemFaultStatus()
        self._set_stamp(fault, now_ns)
        fault.fault_state = fault_state

        teleop = TwistStamped()
        teleop.header.frame_id = "base_link"
        self._set_stamp(teleop, now_ns)
        teleop.twist.linear.x = 0.4

        self.fault.publish(fault)
        self.teleop.publish(teleop)

    def _scan_message(
        self,
        *,
        distance_m: float = 5.0,
        fill_value: float | None = None,
        ghost_obstacle: bool = False,
        frame_id: str = "laser",
        timestamp_ns: int | None = None,
    ) -> LaserScan:
        scan = LaserScan()
        scan.header.frame_id = frame_id
        effective_timestamp_ns = (
            self._next_scan_timestamp_ns()
            if timestamp_ns is None
            else timestamp_ns
        )
        self._set_stamp(scan, effective_timestamp_ns)
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / 360
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [
            distance_m if fill_value is None else fill_value
        ] * 360
        if ghost_obstacle:
            scan.ranges[180] = 0.2
        return scan

    def _publish_frame(
        self,
        *,
        distance_m: float = 5.0,
        fault_state: int = SystemFaultStatus.FAULT_OK,
    ) -> None:
        self._publish_control(fault_state)
        self.scan.publish(self._scan_message(distance_m=distance_m))

    def _clear_observations(self) -> None:
        self.envelopes.clear()
        self.diagnostics.clear()
        self.outputs.clear()
        self.statuses.clear()

    def _recover_nominal(self) -> None:
        self._clear_observations()
        self._spin_until(
            lambda: (
                any(
                    message.state == message.STATE_NOMINAL
                    and message.reason == "NOMINAL_AUTHORITY"
                    for message in self.diagnostics
                )
                and any(
                    message.max_linear_x_mps > 0.0
                    for message in self.envelopes
                )
                and any(
                    message.twist.linear.x > 0.0 for message in self.outputs
                )
            ),
            on_cycle=self._periodic_stream(self._publish_frame),
        )

    def _find_correlated_stop(
        self,
        *,
        diagnostics_reason: str,
        arbitration_reason: str = "MOTION_ENVELOPE_STOP_ONLY",
    ) -> (
        tuple[
            EnvelopeDiagnostics,
            PermittedMotionEnvelope,
            ArbitrationStatus,
        ]
        | None
    ):
        for candidate in self.diagnostics:
            if candidate.reason != diagnostics_reason:
                continue
            envelope = next(
                (
                    message
                    for message in self.envelopes
                    if message.correlation_id == candidate.correlation_id
                ),
                None,
            )
            status = next(
                (
                    message
                    for message in self.statuses
                    if (
                        message.motion_envelope_correlation_id
                        == candidate.correlation_id
                        and message.rejection_reason == arbitration_reason
                    )
                ),
                None,
            )
            if envelope is not None and status is not None:
                return candidate, envelope, status
        return None

    def _assert_stop(
        self,
        *,
        diagnostics_reason: str,
        diagnostics_state: int,
        arbitration_reason: str = "MOTION_ENVELOPE_STOP_ONLY",
    ) -> EnvelopeDiagnostics:
        matched = self._find_correlated_stop(
            diagnostics_reason=diagnostics_reason,
            arbitration_reason=arbitration_reason,
        )
        self.assertIsNotNone(
            matched,
            "No correlated diagnostic, envelope, and arbitration stop found",
        )
        assert matched is not None
        diagnostic, envelope, status = matched
        self.assertEqual(diagnostic.state, diagnostics_state)
        self.assertEqual(envelope.max_linear_x_mps, 0.0)
        self.assertEqual(envelope.max_angular_z_radps, 0.0)
        self.assertEqual(status.mode, status.MODE_FORCED_ZERO)
        self.assertEqual(status.commanded_twist.linear.x, 0.0)
        self.assertEqual(status.commanded_twist.angular.z, 0.0)
        self.assertTrue(
            any(
                message.twist.linear.x == 0.0
                and message.twist.angular.z == 0.0
                for message in self.outputs
            )
        )

        propagation_ns = _stamp_ns(status) - _stamp_ns(envelope)
        self.assertGreaterEqual(propagation_ns, 0)
        self.assertLessEqual(propagation_ns, _CHAIN_STOP_BUDGET_NS)
        return diagnostic

    def _wait_for_stop(
        self,
        *,
        diagnostics_reason: str,
        arbitration_reason: str = "MOTION_ENVELOPE_STOP_ONLY",
        on_cycle: Callable[[], None] | None = None,
        timeout_sec: float = _SCENARIO_TIMEOUT_SEC,
    ) -> None:
        self._spin_until(
            lambda: (
                self._find_correlated_stop(
                    diagnostics_reason=diagnostics_reason,
                    arbitration_reason=arbitration_reason,
                )
                is not None
                and any(
                    message.twist.linear.x == 0.0
                    and message.twist.angular.z == 0.0
                    for message in self.outputs
                )
            ),
            timeout_sec=timeout_sec,
            on_cycle=on_cycle,
        )

    def test_fault_injection_suite(self) -> None:
        """Prove fail-closed propagation for every TB-EVAL-008C fault."""
        self._wait_for_graph()
        self._recover_nominal()

        with self.subTest("lidar signal loss"):
            self._clear_observations()
            self._wait_for_stop(
                diagnostics_reason="SCAN_TIMEOUT",
                on_cycle=self._periodic_stream(self._publish_control),
            )
            diagnostic = self._assert_stop(
                diagnostics_reason="SCAN_TIMEOUT",
                diagnostics_state=EnvelopeDiagnostics.STATE_FAIL_CLOSED,
            )
            self.assertGreater(diagnostic.scan_age_sec, _SENSOR_TIMEOUT_SEC)
            self._recover_nominal()

        with self.subTest("single-beam ghost obstacle"):
            self._clear_observations()
            injected_scan = self._scan_message(ghost_obstacle=True)
            injected_at_ns = _stamp_ns(injected_scan)
            self.scan.publish(injected_scan)
            self._publish_control()
            self._wait_for_stop(
                diagnostics_reason="CLEARANCE_REQUIRES_STOP",
                on_cycle=self._periodic_stream(self._publish_control),
                timeout_sec=1.0,
            )
            diagnostic = self._assert_stop(
                diagnostics_reason="CLEARANCE_REQUIRES_STOP",
                diagnostics_state=(
                    EnvelopeDiagnostics.STATE_PROTECTIVE_STOP
                ),
            )
            reaction_ns = _stamp_ns(diagnostic) - injected_at_ns
            self.assertGreaterEqual(reaction_ns, 0)
            self.assertLessEqual(
                reaction_ns,
                _ENVELOPE_REACTION_BUDGET_NS,
            )
            self._recover_nominal()

        for label, value, reason in (
            ("nan flood", math.nan, "SCAN_RANGE_INVALID"),
            ("positive infinity flood", math.inf, "SCAN_NO_FINITE_RETURNS"),
        ):
            with self.subTest(label):
                self._clear_observations()
                self.scan.publish(self._scan_message(fill_value=value))
                self._publish_control()
                self._wait_for_stop(
                    diagnostics_reason=reason,
                    on_cycle=self._periodic_stream(self._publish_control),
                    timeout_sec=1.0,
                )
                self._assert_stop(
                    diagnostics_reason=reason,
                    diagnostics_state=EnvelopeDiagnostics.STATE_FAIL_CLOSED,
                )
                self._recover_nominal()

        with self.subTest("invalid frame id"):
            self._clear_observations()
            self.scan.publish(self._scan_message(frame_id=" "))
            self._publish_control()
            self._wait_for_stop(
                diagnostics_reason="SCAN_FRAME_MISSING",
                on_cycle=self._periodic_stream(self._publish_control),
                timeout_sec=1.0,
            )
            self._assert_stop(
                diagnostics_reason="SCAN_FRAME_MISSING",
                diagnostics_state=EnvelopeDiagnostics.STATE_FAIL_CLOSED,
            )
            self._recover_nominal()

        with self.subTest("system fault overrides clear scan"):
            self._clear_observations()

            def publish_hardware_fault() -> None:
                self._publish_frame(
                    fault_state=SystemFaultStatus.FAULT_HARDWARE_FAULT
                )

            self._wait_for_stop(
                diagnostics_reason="SYSTEM_FAULT_HARDWARE_FAULT",
                arbitration_reason="SYSTEM_FAULT_HARDWARE_FAULT",
                on_cycle=self._periodic_stream(publish_hardware_fault),
            )
            self._assert_stop(
                diagnostics_reason="SYSTEM_FAULT_HARDWARE_FAULT",
                diagnostics_state=EnvelopeDiagnostics.STATE_FAIL_CLOSED,
                arbitration_reason="SYSTEM_FAULT_HARDWARE_FAULT",
            )
            self._recover_nominal()

        with self.subTest("sticky scan clock regression"):
            self._clear_observations()
            regressed_timestamp_ns = self.last_scan_timestamp_ns - 1
            self.scan.publish(
                self._scan_message(timestamp_ns=regressed_timestamp_ns)
            )
            self._publish_control()
            self._wait_for_stop(
                diagnostics_reason="CLOCK_REGRESSION_LATCHED",
                on_cycle=self._periodic_stream(self._publish_control),
                timeout_sec=1.0,
            )
            latched = self._assert_stop(
                diagnostics_reason="CLOCK_REGRESSION_LATCHED",
                diagnostics_state=EnvelopeDiagnostics.STATE_FAIL_CLOSED,
            )

            self._clear_observations()
            self._publish_frame()
            self._wait_for_stop(
                diagnostics_reason="CLOCK_REGRESSION_LATCHED",
                on_cycle=self._periodic_stream(self._publish_control),
                timeout_sec=1.0,
            )
            persisted = self._assert_stop(
                diagnostics_reason="CLOCK_REGRESSION_LATCHED",
                diagnostics_state=EnvelopeDiagnostics.STATE_FAIL_CLOSED,
            )
            self.assertGreater(persisted.sequence_id, latched.sequence_id)


@launch_testing.post_shutdown_test()
class TestDynamicEnvelopeFaultInjectionShutdown(unittest.TestCase):
    """Require every launched control-plane process to exit successfully."""

    def test_exit_codes(self, proc_info: object) -> None:
        launch_testing.asserts.assertExitCodes(proc_info)
