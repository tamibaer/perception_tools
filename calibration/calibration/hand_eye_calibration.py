import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

from scipy.spatial.transform import Rotation as R
import numpy as np


class SequentialPoses(Node):

    def __init__(self):
        super().__init__('sequential_poses')

        self.client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info("Sequential MoveIt2 Node initialized")


    def wait_for_server(self):
        self.client.wait_for_server()


    from geometry_msgs.msg import Pose

    def circle_poses(self):
        q = R.from_euler('xyz', [np.pi, 0.0, 0.0]).as_quat()
        start_pose = Pose()
        start_pose.position.x = 0.0060
        start_pose.position.y = 0.4735
        start_pose.position.z = 0.2948
        start_pose.orientation.x = q[0]
        start_pose.orientation.y = q[1]
        start_pose.orientation.z = q[2]
        start_pose.orientation.w = q[3]
        radius = 0.2
        n_points = 4
        center = (0.006, 0.3, 0.3)
        
        cx, cy, cz = center
        poses = []
        poses.append(start_pose)

        for i in range(n_points):
            theta = 2 * np.pi * i / n_points

            p = Pose()
            p.position.x = cx + radius * np.cos(theta)
            p.position.y = cy + radius * np.sin(theta)
            p.position.z = cz

            p.orientation.w = q[0]
            p.orientation.x = q[1]
            p.orientation.y = q[2]
            p.orientation.z = q[3]

            poses.append(p)

        return poses

    def get_poses(self):

        q = R.from_euler('xyz', [np.pi, 0.0, 0.0]).as_quat()

        poses = []

        p1 = Pose()
        p1.position.x = 0.0060
        p1.position.y = 0.4735
        p1.position.z = 0.2948
        p1.orientation.x = q[0]
        p1.orientation.y = q[1]
        p1.orientation.z = q[2]
        p1.orientation.w = q[3]

        p2 = Pose()
        p2.position.x = 0.3
        p2.position.y = 0.2
        p2.position.z = 0.3
        p2.orientation.x = q[0]
        p2.orientation.y = q[1]
        p2.orientation.z = q[2]
        p2.orientation.w = q[3]

        p3 = Pose()
        p3.position.x = 0.5
        p3.position.y = 0.3
        p3.position.z = 0.3
        p3.orientation.x = q[0]
        p3.orientation.y = q[1]
        p3.orientation.z = q[2]
        p3.orientation.w = q[3]

        poses.append(p1)
        poses.append(p2)
        poses.append(p3)

        return poses

    def pose_to_goal(self, pose):

        goal = MoveGroup.Goal()

        goal.request.group_name = "ur5e_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 0.05
        goal.request.max_acceleration_scaling_factor = 0.05

        goal.request.workspace_parameters.header.frame_id = "ur5e_base_link"

        goal.request.goal_constraints.append(self.to_constraint(pose))

        return goal

    def to_constraint(self, pose):

        c = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = "ur5e_base_link"
        pc.link_name = "ur5e_tool0"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0

        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = "ur5e_base_link"
        oc.link_name = "ur5e_tool0"

        oc.orientation = pose.orientation  

        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1

        oc.weight = 1.0

        c.orientation_constraints.append(oc)

        return c


    def execute(self):

        self.wait_for_server()

        poses = self.get_poses()
        #poses = self.circle_poses()

        self.get_logger().info(f"Executing {len(poses)} poses")

        for i, pose in enumerate(poses):

            self.get_logger().info(f"Moving to Pose {i+1}")

            goal = self.pose_to_goal(pose)

            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("All poses executed")


def main(args=None):

    rclpy.init(args=args)

    node = SequentialPoses()

    try:
        node.execute()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()