# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import time


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.idx_c0 = 0
        self.idx_c1 = 0

        self.i = 0
        self.sum = 0
        self.avg = 0
        # self.time_t = 0
        # self.time_sum = 0
        # self.time_avg = 0
        # self.num = 0



        self.subscription = self.create_subscription(    # the first sub '/odometry/filtered'
            Odometry,
            '/odometry/filtered',
            self.listener_callback,
            10)
        # self.subscription  # prevent unused variable warning

        self.subscription2 = self.create_subscription(    # the second sub '/odom'
            Odometry,
            '/odom',
            self.listener_callback2,
            10)
        # self.subscription2  # prevent unused variable warning

        self.publisher = self.create_publisher(
            String, 
            'evalate', 
            10)
        self.timer = self.create_timer(0.5, self.timer_callback) 




    def listener_callback(self, msg: Odometry):
        # self.get_logger().info('I heard from /odometry/filtered: "%s"' % msg.pose.pose)
        if self.idx_c0 == 0:
            self.idx_c0 = 1   

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_z = msg.pose.pose.position.z
        self.odom_filter_x = odom_x
        self.odom_filter_y = odom_y
        self.odom_filter_z = odom_z

        orien_w = msg.pose.pose.orientation.w
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
        
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_z = msg.pose.pose.position.z
        self.odom_x = odom_x
        self.odom_y = odom_y
        self.odom_z = odom_z

        orien_w = msg.pose.pose.orientation.w
        orien_x = msg.pose.pose.orientation.x
        orien_y = msg.pose.pose.orientation.y
        orien_z = msg.pose.pose.orientation.z
        self.orien_w = orien_w
        self.orien_x = orien_x
        self.orien_y = orien_y
        self.orien_z = orien_z


        # print('/odom: "%s"' % odom_x, odom_y, odom_z)

    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        
        if (self.idx_c0 == 1) and (self.idx_c1 == 1):
            eval = math.sqrt((self.odom_filter_x - self.odom_x)**2 
                           + (self.odom_filter_y - self.odom_y)**2 
                           + (self.odom_filter_z - self.odom_z)**2)
            # msg = String()                                            # 创建一个String类型的消息对象
            # msg.data = str(eval)                                # 填充消息对象中的消息数据
            # self.publisher.publish(msg)                                     # 发布话题消息
            # self.get_logger().info('eval: "%s"' % msg.data)  

            eval2 = 2 * math.acos(abs(self.orien_filter_w * self.orien_w 
                                    + self.orien_filter_x * self.orien_x 
                                    + self.orien_filter_y * self.orien_y 
                                    + self.orien_filter_z * self.orien_z))



            if eval >= 0.001:
                # begin = time.time()

                self.i = self.i + 1
                self.sum = self.sum + eval
                self.avg = self.sum/self.i
                print('/num of eval: "%s"' % self.i)
                print('/sum of eval: "%s"' % self.sum)
                print('/avg of eval: "%s"' % self.avg)

                # end = time.time()
                # self.time_t = end - begin
                # self.time_sum = self.time_sum + self.time_t
                # self.time_avg = self.time_sum/self.i
                # print('/num of time: "%s"' % self.time_t)
                # print('/sum of time: "%s"' % self.time_sum)
                # print('/avg of time: "%s"' % self.time_avg)


                # print(a-b)

            print('/eval: "%s"' % eval)
            print('/eval2: "%s"' % eval2)

        else:
            pass

        # i = 0
        # if eval >= 

    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    