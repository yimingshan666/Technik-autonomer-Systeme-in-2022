# ===================================================================================
# Description: Publish the /goal_pose to the car.
# The car will move according to this goal_pose.
# Only one message is published, so sometimes there may be a problem.
# If something goes wrong, run pub_goal2.
# Modified By: Yiming Shan
# Date: January 30, 2022
# ===================================================================================


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import math
import time


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')

        self.publisher = self.create_publisher(    ## pub the eval...
            PoseStamped, 
            '/goal_pose', 
            10)
        # self.publish_once()
        # self.timer = self.create_timer(0.2, self.timer_callback) 

    def publish_once(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = -6.62
        msg.pose.position.y = 1.887
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.934
        msg.pose.orientation.w = 0.355
        self.publisher.publish(msg)
        # self.get_logger().info("Goal pose published: " % msg)
        # self.get_logger().info("Goal pose published: x=%.1f" % (msg))   
        self.get_logger().info("Goal pose published: posi_x=%.1f, posi_y=%.1f, posi_z=%.1f, \
                                orien_x=%.1f, orien_y=%.1f, orien_z=%.1f, orien_w=%.1f" \
                                % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

    

def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()
    goal_publisher.publish_once() #call the function to publish the message


    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
