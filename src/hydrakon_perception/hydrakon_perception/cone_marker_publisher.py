import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import ObjectsStamped

CONE_COLORS = {
    'yellow_cone': (1.0, 1.0, 0.0),
    'blue_cone': (0.0, 0.3, 1.0),
    'orange_cone': (1.0, 0.45, 0.0),
    'large_orange_cone': (1.0, 0.15, 0.0),
    'unknown_cone': (0.6, 0.6, 0.6),
}

CONE_RADIUS_M = 0.15
CONE_HEIGHT_M = 0.3
LARGE_CONE_SCALE = 1.4

# ZED SDK OBJECT_TRACKING_STATE values (see zed_msgs/Object.msg)
TRACKING_STATE_OK = 1
TRACKING_STATE_SEARCHING = 2


class ConeMarkerPublisher(Node):

    def __init__(self):
        super().__init__('cone_marker_publisher')
        self._prev_marker_ids = set()
        self._sub = self.create_subscription(
            ObjectsStamped, '/zed/zed_node/obj_det/objects', self._on_objects, 10
        )
        self._pub = self.create_publisher(MarkerArray, 'cone_markers', 10)

    def _on_objects(self, msg: ObjectsStamped) -> None:
        marker_array = MarkerArray()
        current_marker_ids = set()

        for obj in msg.objects:
            color = CONE_COLORS.get(obj.label)
            if color is None:
                continue
            if obj.tracking_state not in (TRACKING_STATE_OK, TRACKING_STATE_SEARCHING):
                continue

            marker = Marker()
            marker.header = msg.header
            marker.ns = 'cones'
            marker.id = int(obj.label_id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = float(obj.position[0])
            marker.pose.position.y = float(obj.position[1])
            marker.pose.position.z = float(obj.position[2])
            marker.pose.orientation.w = 1.0

            scale = LARGE_CONE_SCALE if obj.label == 'large_orange_cone' else 1.0
            marker.scale.x = CONE_RADIUS_M * 2.0 * scale
            marker.scale.y = CONE_RADIUS_M * 2.0 * scale
            marker.scale.z = CONE_HEIGHT_M * scale

            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0

            marker_array.markers.append(marker)
            current_marker_ids.add(marker.id)

        for stale_id in self._prev_marker_ids - current_marker_ids:
            delete_marker = Marker()
            delete_marker.header = msg.header
            delete_marker.ns = 'cones'
            delete_marker.id = stale_id
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        self._prev_marker_ids = current_marker_ids
        self._pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
