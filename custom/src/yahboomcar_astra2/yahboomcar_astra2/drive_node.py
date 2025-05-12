# ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt16, String
import os
import sys

# commom lib
import math
import numpy as np
import time
from time import sleep
from yahboomcar_laser.common import *

print("import done")
RAD2DEG = 180 / math.pi


class Driver_Node(Node):
    def __init__(self, name):
        super().__init__(name)
        # create a sub
        self.sub_laser = self.create_subscription(
            LaserScan, "/scan", self.registerScan, 1
        )
        self.sub_JoyState = self.create_subscription(
            Bool, "/JoyState", self.JoyStateCallback, 1
        )
        # create a pub
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.sub_driver = self.create_subscription(
            Twist, "/custom_cmd_vel", self.driver_callback, 1
        )
        self.pub_Buzzer = self.create_publisher(UInt16, "/beep", 1)
        self.publisher_ = self.create_publisher(String, "/driver_status", 1)

        # declareparam
        self.declare_parameter("linear", 0.3)
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.declare_parameter("angular", 1.0)
        self.angular = self.get_parameter("angular").get_parameter_value().double_value
        self.declare_parameter("LaserAngle", 45.0)
        self.LaserAngle = (
            self.get_parameter("LaserAngle").get_parameter_value().double_value
        )
        self.declare_parameter("ResponseDist", 0.55)
        self.ResponseDist = (
            self.get_parameter("ResponseDist").get_parameter_value().double_value
        )
        self.declare_parameter("Switch", False)
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value

        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.Moving = False

        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value
        self.angular = self.get_parameter("angular").get_parameter_value().double_value
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.LaserAngle = (
            self.get_parameter("LaserAngle").get_parameter_value().double_value
        )
        self.ResponseDist = (
            self.get_parameter("ResponseDist").get_parameter_value().double_value
        )

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data

    def buzz(self, state: bool):
        # Create a UInt16 message for the buzzer
        beep_msg = UInt16()
        beep_msg.data = 1 if state else 0

        # Publish the message to the "/beep" topic
        self.pub_Buzzer.publish(beep_msg)

        # Log the buzzer state being published
        #self.get_logger().info(f"Publishing Buzzer State: {beep_msg.data}")

    def publish_status(self, msg: str):
        # The status message
        status_msg = String()

        # You can customize the status text here
        status_msg.data = msg

        # Publish the message to the "/status" topic
        self.publisher_.publish(status_msg)

        # Log the status being published
        #self.get_logger().info(f"Publishing: {status_msg.data}")

    def driver_callback(self, msg):
        if not isinstance(msg, Twist):
            return
        if self.Joy_active:
            return
        if self.Switch:
            return
        self.Moving = True
        #self.get_logger().info(f"Publishing: {msg.linear.x} {msg.angular.z}")
        self.safety_check(msg)

    def safety_check(self, msg):
        if self.Joy_active or self.Switch == True:
            if self.Moving == True:
                print("stop")
                self.publish_status(
                    "Joy active" if self.Joy_active else "Switch active"
                )
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        self.Moving = True
        new_msg = Twist()
        if msg.linear.x > 0 and self.front_warning > 10:
                self.publish_status("Obstacle detected in front")
                new_msg.linear.x = 0.0
        else:
            new_msg.linear.x = msg.linear.x    
            
        if msg.angular.z > 0 and self.Left_warning > 10:
                self.publish_status("Obstacle detected on left")
                new_msg.angular.z = 0.0
        elif msg.angular.z < 0 and self.Right_warning > 10:
                self.publish_status("Obstacle detected on right")
                new_msg.angular.z = 0.0
        else:
            new_msg.angular.z = msg.angular.z

        if msg.linear.x == new_msg.linear.x and msg.angular.z == new_msg.angular.z:
            self.buzz(False)
        else:
            self.buzz(True)

        #self.get_logger().info(f"Publishing: Final {new_msg.linear.x} {new_msg.angular.z}")
        self.pub_vel.publish(new_msg)

    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 + cmd2
        os.system(cmd)

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return
        ranges = np.array(scan_data.ranges)
        Right_warning = 0
        Left_warning = 0
        front_warning = 0

        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180:
                angle = angle - 360
            # 60 means that the range of radar detection is set to 120 degrees
            if 20 < angle < self.LaserAngle:
                if ranges[i] < self.ResponseDist * 1.5:
                    Left_warning += 1
            if -self.LaserAngle < angle < -20:
                if ranges[i] < self.ResponseDist * 1.5:
                    Right_warning += 1
            if abs(angle) <= 20:
                if ranges[i] <= self.ResponseDist * 1.5:
                    front_warning += 1
        self.Right_warning = Right_warning
        self.Left_warning = Left_warning
        self.front_warning = front_warning


def main():
    rclpy.init()
    driver_Node = Driver_Node("driver_node")
    print("start driver node")
    try:
        rclpy.spin(driver_Node)
    except KeyboardInterrupt:
        pass
    finally:
        driver_Node.exit_pro()
        # rclpy.shutdown()
