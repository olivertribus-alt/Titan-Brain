"""ROS 2 Jazzy transport wrapper for Titan Brain safety observations."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from pydantic import ValidationError
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from titan_brain_msgs.msg import (
    DirectionalSafetyObservation as DirectionalSafetyObservationMsg,
)
from titan_brain_msgs.msg import SafetyEvaluationStatus as SafetyEvaluationStatusMsg
from titan_brain_msgs.msg import SafetyObservation as SafetyObservationMsg
from titan_brain_msgs.msg import SafetyStabilityStatus as SafetyStabilityStatusMsg

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
from core.braking import BrakingEnvelopeConfig
from core.incident_store import FileIncidentStore, IncidentStoreError
from core.safety import SafetyRuleConfig
from core.stability import EvaluatorState, StabilityConfig, StabilityTransition
from core.types.incident import Pose2D

_STATUS_SCHEMA_VERSION = "0.1"
_STABILITY_STATUS_SCHEMA_VERSION = "0.1"
ObservationMessage = SafetyObservationMsg | DirectionalSafetyObservationMsg


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


def _required_text_parameter(node: Node, name: str) -> str:
    value = node.declare_parameter(name, Parameter.Type.STRING).value
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"ROS parameter {name!r} must be a non-blank string")
    return value


def _required_finite_parameter(
    node: Node,
    name: str,
    *,
    allow_zero: bool,
) -> float:
    value = node.declare_parameter(name, Parameter.Type.DOUBLE).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"ROS parameter {name!r} must be numeric")
    checked = float(value)
    minimum_is_valid = checked >= 0.0 if allow_zero else checked > 0.0
    if not math.isfinite(checked) or not minimum_is_valid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(
            f"ROS parameter {name!r} must be finite and {qualifier}"
        )
    return checked


def _required_probability_parameter(node: Node, name: str) -> float:
    value = _required_finite_parameter(node, name, allow_zero=True)
    if value > 1.0:
        raise ValueError(f"ROS parameter {name!r} must be at most 1.0")
    return value


def _required_positive_integer_parameter(node: Node, name: str) -> int:
    value = node.declare_parameter(name, Parameter.Type.INTEGER).value
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"ROS parameter {name!r} must be a positive integer")
    return value


def _stability_state_code(transition: StabilityTransition) -> int:
    return {
        EvaluatorState.OK: SafetyStabilityStatusMsg.STATE_OK,
        EvaluatorState.WARNING: SafetyStabilityStatusMsg.STATE_WARNING,
        EvaluatorState.E_STOP: SafetyStabilityStatusMsg.STATE_E_STOP,
        EvaluatorState.RECOVERY_HOLDING: (
            SafetyStabilityStatusMsg.STATE_RECOVERY_HOLDING
        ),
    }[transition.state]


class SafetyObservationNode(Node):
    """Validate, transform, evaluate, and publish safety transport status."""

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__(
            "safety_observation_node",
            parameter_overrides=parameter_overrides,
        )

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

        dynamic_braking_enabled = bool(
            self.declare_parameter("dynamic_braking_enabled", False).value
        )
        safety_rules: SafetyRuleConfig | None = None
        if dynamic_braking_enabled:
            safety_rules = SafetyRuleConfig(
                policy_version=_required_text_parameter(
                    self,
                    "safety_policy_version",
                ),
                clearance_threshold_m=_required_finite_parameter(
                    self,
                    "clearance_threshold_m",
                    allow_zero=False,
                ),
                confidence_threshold=_required_probability_parameter(
                    self,
                    "confidence_threshold",
                ),
                braking_envelope=BrakingEnvelopeConfig(
                    policy_version=_required_text_parameter(
                        self,
                        "braking_policy_version",
                    ),
                    reaction_time_ns=_required_positive_integer_parameter(
                        self,
                        "reaction_time_ns",
                    ),
                    assured_deceleration_mps2=_required_finite_parameter(
                        self,
                        "assured_deceleration_mps2",
                        allow_zero=False,
                    ),
                    clearance_margin_m=_required_finite_parameter(
                        self,
                        "clearance_margin_m",
                        allow_zero=True,
                    ),
                ),
            )

        stability_enabled = bool(
            self.declare_parameter("stability_enabled", False).value
        )
        stability_config: StabilityConfig | None = None
        if stability_enabled:
            stability_config = StabilityConfig(
                policy_version=_required_text_parameter(
                    self,
                    "stability_policy_version",
                ),
                clearance_hysteresis_m=_required_finite_parameter(
                    self,
                    "clearance_hysteresis_m",
                    allow_zero=True,
                ),
                recovery_hold_time_ns=_seconds_to_ns(
                    _required_finite_parameter(
                        self,
                        "recovery_hold_time_s",
                        allow_zero=False,
                    ),
                    name="recovery_hold_time_s",
                ),
            )

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
        self._stability_config = stability_config
        self._tf_timeout = Duration(
            nanoseconds=_seconds_to_ns(tf_timeout_sec, name="tf_timeout_sec")
        )
        self._adapter = RosObservationAdapter(
            FileIncidentStore(Path(incident_store_path)),
            config=adapter_config,
            safety_rules=safety_rules,
            stability_config=stability_config,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._status_publisher = self.create_publisher(
            SafetyEvaluationStatusMsg,
            "/safety/evaluation_status",
            status_qos_profile(),
        )
        self._stability_status_publisher = self.create_publisher(
            SafetyStabilityStatusMsg,
            "/safety/stability_status",
            status_qos_profile(),
        )
        self._observation_subscription = self.create_subscription(
            SafetyObservationMsg,
            "/safety/observation",
            self._on_observation,
            sensor_data_qos_profile(),
        )
        self._directional_observation_subscription = self.create_subscription(
            DirectionalSafetyObservationMsg,
            "/safety/directional_observation",
            self._on_directional_observation,
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
        message: ObservationMessage,
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
        message: ObservationMessage,
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
            "directional_data": None,
        }

    def _normalized_directional_payload(
        self,
        message: DirectionalSafetyObservationMsg,
        pose: Pose2D,
        frame_id: str,
    ) -> dict[str, object]:
        ignored_velocity_components = (
            float(message.velocity.linear.z),
            float(message.velocity.angular.x),
            float(message.velocity.angular.y),
        )
        if any(component != 0.0 for component in ignored_velocity_components):
            raise ValueError(
                "Directional observation contains unsupported 3D velocity"
            )
        payload = self._normalized_payload(message, pose, frame_id)
        payload["directional_data"] = {
            "clearances": {
                "forward_m": float(message.forward_clearance_m),
                "reverse_m": float(message.reverse_clearance_m),
                "left_m": float(message.left_clearance_m),
                "right_m": float(message.right_clearance_m),
            },
            "velocity": {
                "linear_x_mps": float(message.velocity.linear.x),
                "linear_y_mps": float(message.velocity.linear.y),
                "angular_z_radps": float(message.velocity.angular.z),
            },
        }
        return payload

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
        self._publish_stability_status(now, result)

    def _publish_stability_status(
        self,
        now: Time,
        result: RosObservationProcessingResult,
    ) -> None:
        transition = result.stability_transition
        effective = result.decision
        config = self._stability_config
        if transition is None or effective is None or config is None:
            return

        status = SafetyStabilityStatusMsg()
        status.header.stamp = now.to_msg()
        status.header.frame_id = self._target_frame
        status.schema_version = _STABILITY_STATUS_SCHEMA_VERSION
        status.state = _stability_state_code(transition)
        status.reason = transition.reason.value
        status.instantaneous_action = (
            result.instantaneous_decision.decision.action
            if result.instantaneous_decision is not None
            else effective.decision.action
        )
        status.effective_action = effective.decision.action
        status.recovery_active = (
            transition.state is EvaluatorState.RECOVERY_HOLDING
        )
        status.hold_elapsed_ns = transition.hold_elapsed_ns or 0
        status.recovery_hold_time_ns = config.recovery_hold_time_ns
        status.has_release_threshold = transition.release_threshold_m is not None
        status.release_threshold_m = transition.release_threshold_m or 0.0
        self._stability_status_publisher.publish(status)

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
        self._process_observation(message)

    def _on_directional_observation(
        self,
        message: DirectionalSafetyObservationMsg,
    ) -> None:
        self._process_observation(message, directional_message=message)

    def _process_observation(
        self,
        message: ObservationMessage,
        *,
        directional_message: DirectionalSafetyObservationMsg | None = None,
    ) -> None:
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
            if directional_message is not None:
                payload = self._normalized_directional_payload(
                    directional_message,
                    pose,
                    frame_id,
                )
            else:
                payload = self._normalized_payload(message, pose, frame_id)
            result = self._adapter.process(
                payload,
                now_ns=now.nanoseconds,
            )
        except (ValidationError, ValueError) as error:
            self.get_logger().warning(f"Observation payload rejected: {error}")
            self._publish_transport_failure(
                now,
                adapter_status="invalid_observation",
                detail="Observation contains invalid directional data.",
            )
            return
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
