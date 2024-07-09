import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist

import math
import tf_transformations, tf2_ros, tf2_geometry_msgs
from nav_msgs.msg import Odometry


class MultiRobotController(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Here I am assuming only 3 robiles but the launch file can spawn n number of robiles in Gazebo
        self.robile1_vel_pub = self.create_publisher(Twist, "/robile_1/cmd_vel", 10)
        self.robile2_vel_pub = self.create_publisher(Twist, "/robile_2/cmd_vel", 10)
        self.robile3_vel_pub = self.create_publisher(Twist, "/robile_3/cmd_vel", 10)

        self.curr_pose_robile1_sub = self.create_subscription(Odometry, "/robile_1/odom", self.odom_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goalPoseCallback, 10)
        self.goal_pose_received = False
        self.goal_pose = PoseStamped()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def goalPoseCallback(self, msg):
        # Here I am subscribing goal pose given from RViz. Due to time constraint, I am always giving goal in the straight line to the robile1
        self.goal_pose_received = True
        self.goal_pose = msg

    def odom_callback(self, msg):

        self.curr_pose = msg.pose.pose

        dx = self.goal_pose.pose.position.x - self.curr_pose.position.x
        dy = self.goal_pose.pose.position.y - self.curr_pose.position.y

        self.dist_to_goal = math.sqrt((dx ** 2) + (dy ** 2))
        vel_cmd = Twist()

        # The following approach is to move the robile in a straight line assuming the goal is in straight line to the robile 1
        if (self.dist_to_goal > 0.5) and self.goal_pose_received:
            vel_cmd.linear.x = 0.5
        else:
            vel_cmd.linear.x = 0.0

        # I am assigning same velocities to all 3 robots. So when robile1 reaches goal and stops, other robiles will also stop
        # maintaining contant distance among them
        self.robile1_vel_pub.publish(vel_cmd)
        self.robile2_vel_pub.publish(vel_cmd)
        self.robile3_vel_pub.publish(vel_cmd)


def main(args=None):
    rclpy.init(args=args)
    controller = MultiRobotController("pf_planner")
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
