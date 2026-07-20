# project_pkg/project_pkg/fake_detection_publisher.py
import rclpy
from rclpy.node import Node
import numpy as np

from project_interfaces.msg import DetectedROSObject, DetectedROSObjectArray
from project_pkg.conversions import corners_to_ros
from project_pkg.kalman_tracker import reconstruct_corners


class FakeDetectionPublisher(Node):
    '''
    Publishes a synthetic moving object on /detected_objects at a fixed rate, so
    ObjectTrackerNode can be tested end-to-end without a robot or real detector.
    '''

    def __init__(self):
        super().__init__('fake_detection_publisher')
        self.publisher = self.create_publisher(DetectedROSObjectArray, '/detected_objects', 10)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.publish_frame)
        self.t = 0.0

        # one object moving diagonally at constant velocity -- same idea as your notebook's GroundTruthObject
        self.start_pos = np.array([0.0, 0.0])
        self.velocity = np.array([0.5, 0.3])
        self.heading = np.deg2rad(30)
        self.length = 4.0
        self.width = 2.0

    def publish_frame(self):
        true_pos = self.start_pos + self.velocity * self.t
        noisy_pos = true_pos + np.random.normal(0, 0.05, 2)

        corners = reconstruct_corners(noisy_pos, self.heading, self.length, self.width)

        msg = DetectedROSObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        obj = DetectedROSObject()
        obj.id = 0
        obj.center.x = float(noisy_pos[0])
        obj.center.y = float(noisy_pos[1])
        obj.heading = float(self.heading)
        obj.length = float(self.length)
        obj.width = float(self.width)
        obj.corners = corners_to_ros(corners)
        msg.objects.append(obj)

        self.publisher.publish(msg)
        self.get_logger().info(f't={self.t:.1f}s  published detection at ({noisy_pos[0]:.2f}, {noisy_pos[1]:.2f})')

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()