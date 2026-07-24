"""Integration test for ROS 2 Multi-Sensor Envelope Node."""

import unittest
import rclpy
from titan_brain_ros.multi_sensor_envelope_node import MultiSensorEnvelopeNode


class TestMultiSensorEnvelopeNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_node_creation_and_spin(self) -> None:
        node = MultiSensorEnvelopeNode()
        assert node.get_name() == "multi_sensor_envelope_node"
        node.destroy_node()


if __name__ == "__main__":
    unittest.main()
