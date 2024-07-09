import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from tf2_msgs.msg import TFMessage
import numpy as np
import math
import tf_transformations, tf2_ros, tf2_geometry_msgs
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Odometry


class MultiRobotController(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.robot_cmd_vel=Twist()
        self.goal_pose_base_link=PoseStamped()
        self.curr_pose_base_link=PoseStamped()

        #Here I am assuming only 3 robiles but the launch file can spawn n number of robiles in Gazebo
        self.robile1_vel_pub=self.create_publisher(Twist,"/robile_1/cmd_vel",10)
        self.robile2_vel_pub=self.create_publisher(Twist,"/robile_2/cmd_vel",10)
        self.robile3_vel_pub=self.create_publisher(Twist,"/robile_3/cmd_vel",10)

        self.curr_pose_robile1_sub = self.create_subscription(Odometry,"/robile_1/odom",self.odom_callback,10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goalPoseCallback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def goalPoseCallback(self, msg):
        #Here I am subscribing goal pose given from RViz. Due to time constraint, I am always giving goal in the straight line to the robile1
        self.goal_pose_received=True
        self.goal_pose=msg
        self.goal_pose_base_link.header.frame_id='base_link'
        if self.goal_pose is None:
            self.get_logger().info('Goal pose not received yet.')
            return
        
        try:
            trans = self.tf_buffer.lookup_transform('base_laser_front_link', 'odom', rclpy.time.Time())
            self.goal_pose_base_link.pose = do_transform_pose(self.goal_pose.pose, trans)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to transform goal pose to base_laser_front_ink frame")
            return
    
    def odom_callback(self,msg):

        self.curr_pose=msg.pose.pose 
        self.curr_pose_base_link.header.frame_id='base_link'

        #self.get_logger().info(f'Curr pose before transform {self.curr_pose.position}')

        self.curr_pose_base_link.header.frame_id='base_link'
        if self.curr_pose is None:
            self.get_logger().info('Goal pose not received yet.')
            return
        
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            #self.get_logger().info(f'Transform: {trans}')
            self.curr_pose_base_link.pose = do_transform_pose(self.curr_pose, trans)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to transform current pose to base_laser_front_ink frame")
            return
        dx=self.goal_pose_base_link.pose.position.x-self.curr_pose_base_link.pose.position.x
        dy=self.goal_pose_base_link.pose.position.y-self.curr_pose_base_link.pose.position.y

        self.dist_to_goal=math.sqrt((dx**2)+(dy**2))
        vel_cmd=Twist()

        #The following approach is to move the robile in a straight line assuming the goal is in straight line to the robile 1
        if(self.dist_to_goal>0.05):
            vel_cmd.linear.x=0.5
        else:
            vel_cmd.linear.x=0.0

        #I am assigning same velocities to all 3 robots. So when robile1 reaches goal and stops, other robiles will also stop 
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
