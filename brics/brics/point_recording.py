import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import json
from geometry_msgs.msg import TransformStamped
import time

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('target_frame', 'tool0')  # Frame to track (e.g., 'tool0' for UR end effector)
        self.declare_parameter('reference_frame', 'base_link')  # Reference frame (e.g., 'base_link')

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Waypoints storage
        self.waypoints = []

        # Start recording waypoints every second
        self.timer = self.create_timer(1.0, self.record_waypoint)

    def record_waypoint(self):
        try:
            # Look up the transform from the reference frame to the target frame
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time()
            )
            self.log_transform(transform)
            self.waypoints.append(self.transform_to_dict(transform))

        except Exception as e:
            self.get_logger().warn(f"Could not transform from {self.reference_frame} to {self.target_frame}: {e}")

    def log_transform(self, transform: TransformStamped):
        # Log the transformation
        t = transform.transform.translation
        r = transform.transform.rotation
        self.get_logger().info(
            f"Recorded waypoint - Translation: ({t.x:.2f}, {t.y:.2f}, {t.z:.2f}), "
            f"Rotation: ({r.x:.2f}, {r.y:.2f}, {r.z:.2f}, {r.w:.2f})"
        )

    def transform_to_dict(self, transform: TransformStamped):
        """Convert a TransformStamped message to a dictionary for JSON serialization."""
        return {
            'timestamp': transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9,
            'translation': {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'z': transform.transform.translation.z,
            },
            'rotation': {
                'x': transform.transform.rotation.x,
                'y': transform.transform.rotation.y,
                'z': transform.transform.rotation.z,
                'w': transform.transform.rotation.w,
            }
        }

    def save_waypoints_to_json(self, filename='tf2.json'):
        """Save waypoints to a JSON file."""
        with open(filename, 'w') as f:
            json.dump(self.waypoints, f, indent=4)
        self.get_logger().info(f"Saved waypoints to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save waypoints to JSON file when exiting
        node.save_waypoints_to_json('tf2.json')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
