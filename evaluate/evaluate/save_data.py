# ===================================================================================
# Description: Store the evaluation data in the data folder.
# Modified By: Yiming Shan
# Date: January 31, 2022
# ===================================================================================

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import time
import matplotlib.pyplot as plt
import numpy as np
import csv


class Savedata(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.idx_c0 = 0   ## chack if the code running
        self.idx_c1 = 0

        self.i = 0    ## eval the position
        self.sum_posi = 0
        self.avg_posi = 0

        self.sum_orien = 0  ## eval the orientation
        self.avg_orien = 0

        self.hx_avg_posi = []
        self.hx_avg_orien = []
        self.num = []


        self.file = open("/home/user/ros2_ws/src/evaluate/data/ekf_results.csv", "w") 
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp", "position_error", "orientation_error"])


        self.subscription = self.create_subscription(    # the first sub '/odometry/filtered'  after the ukf
            Odometry,
            '/odometry/filtered',
            self.listener_callback,
            10)
        # self.subscription  # prevent unused variable warning

        self.subscription2 = self.create_subscription(    # the second sub '/odom'  before the ukf
            Odometry,
            '/odom',
            self.listener_callback2,
            10)
        # self.subscription2  # prevent unused variable warning

        self.publisher = self.create_publisher(    ## pub the eval...
            String, 
            'evalate', 
            10)
        self.timer = self.create_timer(0.2, self.timer_callback) 




    def listener_callback(self, msg: Odometry):
        # self.get_logger().info('I heard from /odometry/filtered: "%s"' % msg.pose.pose)
        if self.idx_c0 == 0:
            self.idx_c0 = 1   

        odom_x = msg.pose.pose.position.x    ## the position of /odom/filter
        odom_y = msg.pose.pose.position.y
        odom_z = msg.pose.pose.position.z
        self.odom_filter_x = odom_x
        self.odom_filter_y = odom_y
        self.odom_filter_z = odom_z

        orien_w = msg.pose.pose.orientation.w   ## the orien of /odom/filter
        orien_x = msg.pose.pose.orientation.x
        orien_y = msg.pose.pose.orientation.y
        orien_z = msg.pose.pose.orientation.z
        self.orien_filter_w = orien_w
        self.orien_filter_x = orien_x
        self.orien_filter_y = orien_y
        self.orien_filter_z = orien_z


        # print('/odometry/filtered: "%s"' % odom_x, odom_y, odom_z)

    def listener_callback2(self, msg: Odometry):
        # self.get_logger().info('I heard from /odom: "%s"' % msg.pose.pose)
        if self.idx_c1 == 0:
            self.idx_c1 = 1  
        
        odom_x = msg.pose.pose.position.x    ## the position of /odom
        odom_y = msg.pose.pose.position.y
        odom_z = msg.pose.pose.position.z
        self.odom_x = odom_x
        self.odom_y = odom_y
        self.odom_z = odom_z

        orien_w = msg.pose.pose.orientation.w   ## the orien of /odom
        orien_x = msg.pose.pose.orientation.x
        orien_y = msg.pose.pose.orientation.y
        orien_z = msg.pose.pose.orientation.z
        self.orien_w = orien_w
        self.orien_x = orien_x
        self.orien_y = orien_y
        self.orien_z = orien_z


        # print('/odom: "%s"' % odom_x, odom_y, odom_z)

    def timer_callback(self):                                     # the function for eval 

        
        if (self.idx_c0 == 1) and (self.idx_c1 == 1):
            eval_posi = math.sqrt((self.odom_filter_x - self.odom_x)**2 
                           + (self.odom_filter_y - self.odom_y)**2 
                           + (self.odom_filter_z - self.odom_z)**2)
            # msg = String()                                            
            # msg.data = str(eval)                                
            # self.publisher.publish(msg)                                     
            # self.get_logger().info('eval: "%s"' % msg.data)  

            eval_orien = 2 * math.acos(abs(self.orien_filter_w * self.orien_w 
                                    + self.orien_filter_x * self.orien_x 
                                    + self.orien_filter_y * self.orien_y 
                                    + self.orien_filter_z * self.orien_z))



            if eval_posi >= 0.001:
                # begin = time.time()

                self.i = self.i + 1
                self.sum_posi = self.sum_posi + eval_posi   ## eval the posi
                self.avg_posi = self.sum_posi/self.i
                print('/num of eval: "%s"' % self.i)
                # print('/sum of eval_pisition: "%s"' % self.sum_posi)
                print('/avg of eval_position: "%s"' % self.avg_posi)

                self.sum_orien = self.sum_orien + eval_orien   ## eval the orien
                self.avg_orien = self.sum_orien/self.i
                # print('/num of eval: "%s"' % self.i)  
                # print('/sum of eval_orientation: "%s"' % self.sum_orien)
                print('/avg of eval_orientation: "%s"' % self.avg_orien)


                self.writer.writerow([self.i, self.avg_posi, self.avg_orien])


        else:
            pass 


    

def main(args=None):
    rclpy.init(args=args)
    save_subscriber = Savedata()
    rclpy.spin(save_subscriber)

    save_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
