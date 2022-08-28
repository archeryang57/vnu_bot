#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

class OdomNode(Node):                    # todo: rename class name
    def __init__(self):
        super().__init__("odom") 
        self.get_logger().info("odm start publish")
        # * 建立 Timer ( keep it on publisher )
        self.create_timer(0.5, self.callback_func)
        # * 建立 Publisher
        self.publisher_ = self.create_publisher(Odometry, "/odom", 10)

        self.v1 = 128
        self.v2 = 128

        # 使用ROS param设定参数值
        self.linear_coef = rospy.get_param('~linear_coef', default=100.0)
        self.angular_coef = rospy.get_param('~angular_coef', default=10.0)

        self.wheel_diameter = rospy.get_param('~wheel_diameter', default=0.08)
        self.base_width = rospy.get_param('~base_width', default=0.21)
        self.encoder_ticks_per_rev = rospy.get_param('~encoder_ticks_per_rev', default=1980)

        self.encoder1 = 0
        self.encoder2 = 0
        self.encoder1_prev = 0
        self.encoder2_prev = 0

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

        self.time_prev = rospy.Time.now()

        self.is_first_time = True
        self.encoder1_offset = 0
        self.encoder2_offset = 0


    # * 建立 call back function, 撰寫執行動作, for timer
    def callback_func(self):
        self.get_logger().info("Hello callback!!")

    # * 建立 call back function, 撰寫執行動作, for publisher
    # def callback_func(self):
        # msg = String()
        # msg.data = "message from publisher"
        # self.publisher_.publish(msg)



def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    # * 建立 Node
    node = MyNode()                # todo: rename class

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS
    node.destroy_node()  # optional, it will be destoried when GC.
    rclpy.shutdown()

if __name__=="__main__":
    main()
