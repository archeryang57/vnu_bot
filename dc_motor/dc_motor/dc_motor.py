#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .DFRobot_DC_Motor import DFRobot_DC_Motor_IIC as Board
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

class MotorNode(Node):                 
    def __init__(self):
        super().__init__("dc_motor") 
        self.get_logger().info("Hello DC Motor")
        self.init_board()

        # * 建立 Subscription
        self.create_subscription(Twist, "/cmd_vel", self.callback_func, 10)

        # publish odom
        self.publichser_ = self.create_publisher(Odometry, "/odom", 10)

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

    def init_board(self):
        self.board = Board(1, 0x10)    # Select bus 1, set address to 0x10
        self.board_detect()
        while self.board.begin() != self.board.STA_OK:    # Board begin and check board status
            self.print_board_status()
            print("board begin faild")
            time.sleep(2)

        self.board.set_encoder_enable(self.board.ALL)                 # Set selected DC motor encoder enable
        # self.board.set_encoder_disable(self.board.ALL)              # Set selected DC motor encoder disable
        self.board.set_encoder_reduction_ratio(self.board.ALL, 43)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8

        self.board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ
       
        print("board begin success")

    # * 建立 call back function, 撰寫執行動作, for subscription
    def callback_func(self, twist:Twist):
 
        # VNU Bot 規格
        # 輪距：194mm
        # 輪徑: 67mm
        # 輪周長:  210.5 mm (67 * 3.14159)
        # 輪距  l : 194mm () 兩個輪子的距離
        # 每分鐘速度  21cm*100 rpm = 2100cm = 21 m/m
        # 每秒速度  21/60=0.35 m/s
        # Twist單位 linearX: m/s, angularZ: rad/s

        # Max LinearX:    0.35 m/s
        # Max angularZ: 0.35*2/0.194 = 3.6 rad/s
        forwardRate = 228  #  80rpm / 0.35 = 228.57   #80rpm:let turn works
        turnRate = 28      # 100rpm / 3.6 = 27.78
        m1_forward = twist.linear.x * forwardRate
        m2_forward = m1_forward
        max_frd = 0.35 * forwardRate
        
        turn = twist.angular.z * turnRate
        if turn > 0:  # right turn
            m1_forward = min(max_frd, m1_forward + turn )
            m2_forward = m2_forward - turn / 2
        elif turn < 0:
            m1_forward = m1_forward + turn / 2
            m2_forward = min(max_frd, m2_forward - turn )
        m1_dir = self.board.CW
        m2_dir = self.board.CCW

        if m1_forward < 0 :
            m1_dir = self.board.CCW

        if m2_forward < 0 :
            m2_dir = self.board.CW

        self.board.motor_movement(
            [self.board.M1], 
            m1_dir, 
            abs(m1_forward))    # DC motor 1 movement, orientation clockwise
        self.board.motor_movement(
            [self.board.M2], 
            m2_dir, 
            abs(m2_forward))   # DC motor 2 movement, orientation count-clockwise

        print(f"m1:{m1_forward}  m2:{m2_forward}")

    def board_detect(self):
        l = self.board.detecte()
        print("Board list conform:")
        print(l)

    ''' print last operate status, users can use this variable to determine the result of a function call. '''
    def print_board_status(self):
        if self.board.last_operate_status == self.board.STA_OK:
            print("board status: everything ok")
        elif self.board.last_operate_status == self.board.STA_ERR:
            print("board status: unexpected error")
        elif self.board.last_operate_status == self.board.STA_ERR_DEVICE_NOT_DETECTED:
            print("board status: device not detected")
        elif self.board.last_operate_status == self.board.STA_ERR_PARAMETER:
            print("board status: parameter error, last operate no effective")
        elif self.board.last_operate_status == self.board.STA_ERR_SOFT_VERSION:
            print("board status: unsupport board framware version")


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    # * 建立 Node
    node = MotorNode()               

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS
    node.destroy_node()  # optional, it will be destoried when GC.
    rclpy.shutdown()

if __name__=="__main__":
    main()