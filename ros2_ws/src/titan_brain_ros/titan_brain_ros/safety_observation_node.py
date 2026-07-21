"""ROS 2 Jazzy transport wrapper for Titan Brain safety observations."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from pydantic import ValidationError
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from titan_brain_msgs.msg import SafetyEvaluationStatus as SafetyEvaluationStatusMsg
from titan_brain_msgs.msg import SafetyObservation as SafetyObservationMsg

from core.adapters.ros_geometry import (
    PlanarTransform,
    Quaternion,
    apply_planar_transform,
)
from core.adapters.ros_observation import (
    RosObservationAdapter,
    RosObservationAdapterConfig,
    RosObservationProcessingResult,
    WatchdogReport,
)
from core.incident_store import FileIncidentStore, IncidentStoreError
from core.types.incident import Pose2D

_STATUS_SCHEMA_VERSION = "0.1"


def sensor_data_qos_profile() -> QoSProfile:
    """Return the explicit input QoS contract for normalized sensor data."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos_profile() -> QoSProfile:
    """Return the explicit reliable, non-latched output status contract."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _seconds_to_ns(value: float, *, name: str, allow_zero: bool = False) -> int:
    minimum_is_valid = value >= 0.0 if allow_zero else value > 0.0
    if not math.isfinite(value) or not minimum_is_valid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be a finite {qualifier} number")
    nanoseconds = round(value * 1_000_000_000)
    if not allow_zero and nanoseconds <= 0:
        raise ValueError(f"{name} must round to at least one nanosecond")
    return nanoseconds


class SafetyObservationNode(Node):
    """Validate, transform, evaluate, and publish safety transport status."""

    def __init__(self) -> None:
        super().__init__("safety_observation_node")

        target_frame = str(self.declare_parameter("target_frame", "map").value)
        max_age_sec = float(
            self.declare_parameter("max_observation_age_sec", 0.25).value
        )
        max_future_skew_sec = float(
            self.declare_parameter("max_future_skew_sec", 0.0).value
        )
        watchdog_timeout_sec = float(
            self.declare_parameter("watchdog_timeout_sec", 0.5).value
        )
        timer_period_sec = float(
            self.declare_parameter("timer_period_sec", 0.05).value
        )
        tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.05).value
        )
        incident_store_path = str(
            self.declare_parameter(
                "incident_store_path",
                "/tmp/titan_brain/incidents",
            ).value
        )
        if not incident_store_path.strip():
            raise ValueError("incident_store_path must not be blank")

        timer_period_ns = _seconds_to_ns(
            timer_period_sec,
            name="timer_period_sec",
        )

        adapter_config = RosObservationAdapterConfig(
            expected_frame_id=target_frame,
            max_observation_age_ns=_seconds_to_ns(
                max_age_sec,
                name="max_observation_age_sec",
            ),
            max_future_skew_ns=_seconds_to_ns(
                max_future_skew_sec,
                name="max_future_skew_sec",
                allow_zero=True,
            ),
            watchdog_timeout_ns=_seconds_to_ns(
                watchdog_timeout_sec,
                name="watchdog_timeout_sec",
            ),
        )
        self._target_frame = target_frame
        self._tf_timeout = Duration(
            nanoseconds=_seconds_to_ns(tf_timeout_sec, name="tf_timeout_sec")
        )
        self._adapter = RosObservationAdapter(
            FileIncidentStore(Path(incident_store_path)),
            config=adapter_config,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._status_publisher = self.create_publisher(
            SafetyEvaluationStatusMsg,
            "/safety/evaluation_status",
            status_qos_profile(),
        )
        self._observation_subscription = self.create_subscription(
            SafetyObservationMsg,
            "/safety/observation",
            self._on_observation,
            sensor_data_qos_profile(),
        )
        self._watchdog_timer = self.create_timer(
            timer_period_ns / 1_000_000_000,
            self._on_timer,
        )
        self.get_logger().info(
            f"SafetyObservationNode initialized (target_frame={target_frame!r})"
        )

    @property
    def adapter(self) -> RosObservationAdapter:
        """Expose adapter health for ROS integration tests and diagnostics."""
        return self._adapter

    def _transform_pose(
        self,
        message: SafetyObservationMsg,
    ) -> tuple[Pose2D, str]:
        source_frame = message.header.frame_id
        pose = Pose2D(
            x=float(message.pose.x),
            y=float(message.pose.y),
            yaw=float(message.pose.theta),
        )
        if not source_frame or source_frame == self._target_frame:
            return pose, source_frame

        transform_stamped = self._tf_buffer.lookup_transform(
            self._target_frame,
            source_frame,
            Time.from_msg(message.header.stamp),
            timeout=self._tf_timeout,
        )
        transform = transform_stamped.transform
        planar_transform = PlanarTransform(
            translation_x=float(transform.translation.x),
            translation_y=float(transform.translation.y),
            rotation=Quaternion(
                x=float(transform.rotation.x),
                y=float(transform.rotation.y),
                z=float(transform.rotation.z),
                w=float(transform.rotation.w),
            ),
        )
        return apply_planar_transform(pose, planar_transform), self._target_frame

    def _normalized_payload(
        self,
        message: SafetyObservationMsg,
        pose: Pose2D,
        frame_id: str,
    ) -> dict[str, object]:
        return {
            "header": {
                "stamp": {
                    "sec": int(message.header.stamp.sec),
                    "nanosec": int(message.header.stamp.nanosec),
                },
                "frame_id": frame_id,
            },
            "map_id": message.map_id,
            "pose": pose.model_dump(mode="python"),
            "clearance_m": float(message.clearance_m),
            "confidence": float(message.confidence),
            "sensor_id": message.sensor_id,
        }

    def _base_status(
        self,
        now: Time,
        watchdog: WatchdogReport,
    ) -> SafetyEvaluationStatusMsg:
        status = SafetyEvaluationStatusMsg()
        status.header.stamp = now.to_msg()
        status.header.frame_id = self._target_frame
        status.schema_version = _STATUS_SCHEMA_VERSION
        status.watchdog_status = watchdog.status.value
        status.watchdog_healthy = watchdog.healthy
        status.observation_accepted = False
        status.decision_id = ""
        status.action = ""
        status.rule = ""
        status.is_incident = False
        status.detail = ""
        return status

    def _publish_processing_result(
        self,
        now: Time,
        result: RosObservationProcessingResult,
    ) -> None:
        watchdog = self._adapter.watchdog(now_ns=now.nanoseconds)
        status = self._base_status(now, watchdog)
        status.adapter_status = result.adaptation.status.value
        status.observation_accepted = result.adaptation.accepted
        status.detail = result.adaptation.detail or ""
        if result.decision is not None:
            decision = result.decision.decision
            status.decision_id = decision.decision_id or ""
            status.action = decision.action
            status.rule = decision.rule
            status.is_incident = result.decision.is_incident
        self._status_publisher.publish(status)

    def _publish_transport_failure(
        self,
        now: Time,
        *,
        adapter_status: str,
        detail: str,
    ) -> None:
        watchdog = self._adapter.watchdog(now_ns=now.nanoseconds)
        status = self._base_status(now, watchdog)
        status.adapter_status = adapter_status
        status.detail = detail
        self._status_publisher.publish(status)

    def _on_observation(self, message: SafetyObservationMsg) -> None:
        now = self.get_clock().now()
        try:
            pose, frame_id = self._transform_pose(message)
        except TransformException as error:
            self.get_logger().warning(f"TF2 transform rejected: {error}")
            self._publish_transport_failure(
                now,
                adapter_status="tf_unavailable",
                detail=(
                    f"Unable to transform {message.header.frame_id!r} "
                    f"to {self._target_frame!r}."
                ),
            )
            return
        except (ValidationError, ValueError) as error:
            self.get_logger().warning(f"Observation geometry rejected: {error}")
            self._publish_transport_failure(
                now,
                adapter_status="invalid_geometry",
                detail="Observation contains invalid pose or transform geometry.",
            )
            return

        try:
            result = self._adapter.process(
                self._normalized_payload(message, pose, frame_id),
                now_ns=now.nanoseconds,
            )
        except IncidentStoreError as error:
            self.get_logger().error(f"Incident store failure: {error}")
            self._publish_transport_failure(
                now,
                adapter_status="store_error",
                detail="Unable to persist safety decision evidence.",
            )
            return
        self._publish_processing_result(now, result)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        watchdog = self._adapter.watchdog(now_ns=now.nanoseconds)
        status = self._base_status(now, watchdog)
        status.adapter_status = "watchdog"
        self._status_publisher.publish(status)


def main(args: list[str] | None = None) -> None:
    """Run the ROS 2 node until shutdown."""
    rclpy.init(args=args)
    node = SafetyObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
