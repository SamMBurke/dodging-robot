import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from project_interfaces.msg import TrackedObjectArray

class TrackedObjectsVisualizer(Node):
    '''
    Subscribes to /tracked_objects and republishes each object as visualization_msgs/Marker,
    since RViz2 has no native support for our custom message types. Draws a box outline, an
    ID label, and a velocity arrow per tracked object.
    '''
    def __init__(self):
        super().__init__('tracked_objects_visualizer')

        self.subscription = self.create_subscription(
            TrackedObjectArray,
            '/tracked_objects',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(MarkerArray, 'tracked_objects_markers', 10)
        self._prev_marker_count = 0

    def callback(self, msg):
        marker_array = MarkerArray()

        for obj in msg.objects:
            marker_array.markers.append(self._make_box_marker(obj, msg.header))
            marker_array.markers.append(self._make_label_marker(obj, msg.header))
            marker_array.markers.append(self._make_velocity_marker(obj, msg.header))

        # explicitly delete markers left over from a frame with more tracked objects than this one
        current_count = len(marker_array.markers)
        for i in range(current_count, self._prev_marker_count):
            delete_marker = Marker()
            delete_marker.header = msg.header
            delete_marker.ns = 'tracked_objects'
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)
        self._prev_marker_count = current_count

        self.publisher.publish(marker_array)

    def _make_box_marker(self, obj, header):
        marker = Marker()
        marker.header = header
        marker.ns = 'tracked_objects'
        marker.id = obj.id * 3 + 0  # unique ID for this marker
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # line width

        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.3
        marker.color.a = 1.0

        for corner in list(obj.corners) + [obj.corners[0]]:
            point = Point()
            point.x = corner.x
            point.y = corner.y
            point.z = 0.0

            marker.points.append(point)

        marker.lifetime = Duration(seconds=0.3).to_msg()

        return marker
    
    def _make_label_marker(self, obj, header):
        marker = Marker()
        marker.header = header
        marker.ns = 'tracked_objects'
        marker.id = obj.id * 3 + 1  # unique ID for this marker
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.0

        start = Point(x = obj.center.x, y = obj.center.y, z = 0.1)
        # arrow tip = position 1 second from now at current velocity -- length directly shows speed
        end = Point(x = obj.center.x + obj.velocity.x, y = obj.center.y + obj.velocity.y, z = 0.1)

        marker.points = [start, end]

        marker.lifetime = Duration(seconds=0.3).to_msg()

        return marker
    
def main(args=None):
    rclpy.init(args=args)
    node = TrackedObjectsVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()