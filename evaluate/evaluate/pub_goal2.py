# ===================================================================================
# Description: Publish continuous /goal_pose to the car.
# The car will move according to this goal_pose.
# Modified By: Yiming Shan
# Date: January 30, 2022
# ===================================================================================


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
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

        # msg.pose.position.x = 1.1     ## a different position
        # msg.pose.position.y = -5.66
        # msg.pose.position.z = 0.0
        # msg.pose.orientation.x = 0.0
        # msg.pose.orientation.y = 0.0
        # msg.pose.orientation.z = 0.466
        # msg.pose.orientation.w = 0.8844

        msg.pose.position.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Goal pose published: posi_x=%.1f, posi_y=%.1f, posi_z=%.1f, \
                                orien_x=%.1f, orien_y=%.1f, orien_z=%.1f, orien_w=%.1f" \
                                % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

    

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

