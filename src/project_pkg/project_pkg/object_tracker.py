import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.time import Time

import numpy as np

from project_pkg import math
from project_pkg.conversions import corners_to_ros, corners_from_ros
from project_pkg.kalman_tracker import KalmanTracker
from project_pkg.objects import DetectedObject
from project_interfaces import DetectedROSObjectArray, TrackedObject, TrackedObjectArray


class ObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('object_tracker')

        self.subscription = self.create_subscription(
            DetectedROSObjectArray,
            '/detected_objects',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray,
            '/tracked_objects',
            10
        )

        self.tracker = KalmanTracker(max_age=5, min_hits=3, max_valid_distance=1.5)
        self._last_stamp = None
        self._default_dt = 0.1

    def scan_callback(self, msg):
        dt = self._compute_dt(msg.header.stamp)

        detected_objects = self.convert_to_obj(msg)
        tracks = self.tracker.step(detected_objects, dt)

        ros_msg = self.convert_to_ros(tracks, msg.header)
        self.publisher.publish(ros_msg)

    def _compute_dt(self, stamp):
        current_stamp = Time.from_msg(stamp)
        if self._last_stamp is None:
            dt = self._default_dt
        else:
            dt = (current_stamp - self._last_stamp).nanoseconds / 1e9
            if dt <= 0:
                dt = self._default_dt
    
    def convert_to_obj(self, msg):
        objects = []
        for object_msg in msg.objects:
            obj = DetectedObject(
                id = object_msg.id,
                center = np.array([object_msg.center.x, object_msg.center.y]),
                heading = object_msg.heading,
                length = object_msg.length,
                width = object_msg.width,
                corners = corners_from_ros(object_msg.corners)
            )
            objects.append(obj)

        return objects
    
    def convert_to_ros(self, tracks, header):
        msg = TrackedObjectArray()
        msg.header = header

        for track in tracks:
            ros_obj = TrackedObject()

            ros_obj.id = track.id
            ros_obj.center.x = float(track.center[0])
            ros_obj.center.y = float(track.center[1])
            ros_obj.velocity.x = float(track.velocity[0])
            ros_obj.velocity.y = float(track.velocity[1])
            ros_obj.heading = float(track.heading)
            ros_obj.length = float(track.length)
            ros_obj.width = float(track.width)
            ros_obj.corners = corners_to_ros(track.corners)
            ros_obj.age = track.age
            ros_obj.hits = track.hits

            msg.objects.append(ros_obj)

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = ObjectTrackerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()