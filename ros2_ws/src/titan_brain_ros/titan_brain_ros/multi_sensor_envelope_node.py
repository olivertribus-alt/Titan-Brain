"""
ROS 2 Multi-Sensor Fusion Envelope Node for Titan Brain (TB-EVAL-009C)
"""

import time
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from core.multi_sensor_envelope import MultiSensorEnvelopeEvaluator, SensorReading


class MultiSensorEnvelopeNode(Node):
    """ROS 2 wrapper around MultiSensorEnvelopeEvaluator."""

    def __init__(self) -> None:
        super().__init__("multi_sensor_envelope_node")

        self.declare_parameter("stale_timeout_s", 0.200)
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("eval_rate_hz", 50.0)

        stale_timeout = float(self.get_parameter("stale_timeout_s").value)
        min_conf = float(self.get_parameter("min_confidence").value)
        rate_hz = float(self.get_parameter("eval_rate_hz").value)

        self.evaluator = MultiSensorEnvelopeEvaluator(
            stale_timeout_s=stale_timeout,
            min_confidence=min_conf,
        )

        # Default critical sensor registration
        self.evaluator.register_critical_sensor("lidar_front")

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, 10
        )

        # Publishers
        self.fused_dist_pub = self.create_publisher(Float32, "/safety/fused_min_distance", 10)
        self.diag_pub = self.create_publisher(String, "/safety/fusion_diagnostics", 10)

        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self._evaluate_loop)

    def _scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        min_dist = min(valid_ranges) if valid_ranges else float("inf")
        
        now_s = time.time()
        self.evaluator.update_sensor(
            SensorReading(
                sensor_id="lidar_front",
                distance_m=min_dist,
                timestamp_s=now_s,
                is_critical=True,
                confidence=1.0,
            )
        )

    def _evaluate_loop(self) -> None:
        now_s = time.time()
        result = self.evaluator.evaluate_fusion(now_s)

        dist_msg = Float32()
        dist_msg.data = float(result.fused_distance_m)
        self.fused_dist_pub.publish(dist_msg)

        diag_msg = String()
        diag_msg.data = (
            f"emergency={result.is_emergency} | fused_d={result.fused_distance_m:.2f}m | "
            f"strictest={result.strictest_sensor_id} | active={result.active_sensors_count}"
        )
        self.diag_pub.publish(diag_msg)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MultiSensorEnvelopeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
