import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class ToolPoseTFReader(Node):

    def __init__(self):
        super().__init__('tool_pose_reader')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = "ur5e_base_link"
        self.tool_frame = "ur5e_tool0"

        self.get_logger().info("TF ToolPoseReader initialized")

        self.timer = self.create_timer(1.0, self.read_pose)

    def read_pose(self):

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            self.get_logger().info(
                f"""
                    Tool Pose ({self.tool_frame} → {self.base_frame})
                    Position:
                        x: {t.x:.4f}
                        y: {t.y:.4f}
                        z: {t.z:.4f}
                    Orientation:
                        x: {r.x:.4f}
                        y: {r.y:.4f}
                        z: {r.z:.4f}
                        w: {r.w:.4f}
                    """
                                )

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")


def main(args=None):

    rclpy.init(args=args)

    node = ToolPoseTFReader()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Stopped")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()