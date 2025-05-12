#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboomcar_mediapipe2.media_library import *
from time import sleep, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PoseCtrlArm(Node):
    def __init__(self, name):
        super().__init__(name)
        # self.sub_laser = self.create_subscription(
        #     LaserScan, "/scan", self.registerScan, 1
        # )
        self.pub_Servo1 = self.create_publisher(Int32, "servo_s1", 10)
        self.pub_Servo2 = self.create_publisher(Int32, "servo_s2", 10)
        self.publisher_ = self.create_publisher(String, "/status", 1)
        self.past_listener = 0
        self.timer = self.create_timer(0.001, self.on_timer)
        self.socket = self.create_subscription(
            UInt16, "/listener", self.onSocketCallback, 1
        )
        self.capture = None
        self.frame = None
        # self.x = 0
        # self.y = -45
        # servo1_angle = Int32()
        # servo2_angle = Int32()

        # servo1_angle.data = self.x
        # servo2_angle.data = self.y

        # self.pub_Servo1.publish(servo1_angle)
        # self.pub_Servo2.publish(servo2_angle)
        self.img_flip = False
        self.linear_PID = (20.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.scale = 1000
        self.end = 0
        self.PWMServo_X = 0
        self.PWMServo_Y = -60
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y
        self.PID_init()
        self.filtered_x = 320
        self.filtered_y = 420
        self.filter_alpha = 0.2  # smaller = smoother

        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.pose_detector = Holistic()
        self.hand_detector = HandDetector()
        self.pTime = self.index = 0
        self.media_ros = Media_ROS()
        self.event = threading.Event()
        self.event.set()

    def publish_status(self, msg: str):
        # The status message
        status_msg = String()

        # You can customize the status text here
        status_msg.data = msg

        # Publish the message to the "/status" topic
        self.publisher_.publish(status_msg)

        # Log the status being published
        self.get_logger().info(f"Publishing: {status_msg.data}")

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [
                self.linear_PID[0] / float(self.scale),
                self.linear_PID[0] / float(self.scale),
            ],
            [
                self.linear_PID[1] / float(self.scale),
                self.linear_PID[1] / float(self.scale),
            ],
            [
                self.linear_PID[2] / float(self.scale),
                self.linear_PID[2] / float(self.scale),
            ],
        )
        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)
        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)
        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)

    def follow_hand(self, point_x, point_y):
        # Low-pass filter
        # self.filtered_x = self.filter_alpha * point_x + (1 - self.filter_alpha) * self.filtered_x
        # self.filtered_y = self.filter_alpha * point_y + (1 - self.filter_alpha) * self.filtered_y

        # # Compute error
        # error_x = self.filtered_x - 320
        # error_y = self.filtered_y - 420

        # # Deadband
        # if abs(error_x) < 10:
        #     error_x = 0
        # if abs(error_y) < 10:
        #     error_y = 0

        # [x_Pid, y_Pid] = self.PID_controller.update([error_x, error_y])
        [x_Pid, y_Pid] = self.PID_controller.update([point_x-320, point_y-420])
        print("x_Pid: ", x_Pid)
        print("y_Pid: ", y_Pid)
        # if self.img_flip == True:
        #     self.PWMServo_X += x_Pid
        #     self.PWMServo_Y += y_Pid
        # else:
        #     self.PWMServo_X -= x_Pid
        #     self.PWMServo_Y += y_Pid
        self.PWMServo_X += x_Pid
        self.PWMServo_Y += y_Pid

        if self.PWMServo_X >= 90:
            self.PWMServo_X = 90
        elif self.PWMServo_X <= -90:
            self.PWMServo_X = -90
        if self.PWMServo_Y >= 20:
            self.PWMServo_Y = 20
        elif self.PWMServo_Y <= -90:
            self.PWMServo_Y = -90

        # rospy.loginfo("target_servox: {}, target_servoy: {}".format(self.target_servox, self.target_servoy))
        print("servo1", self.PWMServo_X)
        print("servo2", self.PWMServo_Y)
        servo1_angle = Int32()
        servo1_angle.data = int(self.PWMServo_X)
        servo2_angle = Int32()
        servo2_angle.data = int(self.PWMServo_Y)
        self.pub_Servo1.publish(servo1_angle)
        self.pub_Servo2.publish(servo2_angle)

    def onSocketCallback(self, msg):
        if msg.data != 1:
            if self.capture and self.capture.isOpened():
                self.capture.release()
                print("HandCtrl Camera released.")
            
        if self.past_listener != 1 and msg.data == 1:            
            while(True):
                try:
                    self.capture = cv.VideoCapture(0)
                    if not self.capture.isOpened():
                        raise RuntimeError("Failed to open camera on index 0")

                    self.capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                    self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                    print("Camera opened. FPS:", self.capture.get(cv.CAP_PROP_FPS))
                    break

                except Exception as e:
                    print(f"[ERROR] HandCtrl onSocketCallback: {e}")
                    # Optionally: handle fallback behavior or retry
                    
        self.past_listener = msg.data

    
    def on_timer(self):
        # self.get_param()
        if(self.past_listener != 1):
            return
        ret, self.frame = self.capture.read()
        if not ret:
            return
        # TODO: Get action
        self.process(self.frame)
        action = cv.waitKey(10) & 0xFF
        # NOTE: Added
        if action == ord("q") or action == 113:
            self.capture.release()
            cv.destroyAllWindows()
    
    def process(self, frame):
        frame = cv.flip(frame, 1)
        if self.media_ros.Joy_active:

            frame, lmList, wrist = self.hand_detector.findHands(frame)
            if len(lmList) != 0:
                threading.Thread(
                    target=self.hand_threading, args=(lmList, wrist)
                ).start()
            else:
                self.media_ros.pub_vel(0.0, 0.0, 0.0)

            self.cTime = time()
            fps = 1 / (self.cTime - self.pTime)
            self.pTime = self.cTime
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            self.media_ros.pub_imgMsg(frame)
        return frame

    def go_quadrilateral(self):
        for _ in range(0, 4):
            self.media_ros.pub_vel(0.3, 0.0, 0.0)
            sleep(3)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(1)
            self.media_ros.pub_vel(0.0, 0.0, 1.0)
            sleep(2.2)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
            sleep(1)

    def go_s(self):
        self.media_ros.pub_vel(0.3, 0.0, -0.5)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(1)
        self.media_ros.pub_vel(0.3, 0.0, 0.5)
        sleep(3)
        self.media_ros.pub_vel(0.0, 0.0, 0.0)
        sleep(1)

    def Go_circle(self, flag):
        if flag == 1:
            self.media_ros.pub_vel(0.3, 0.0, 0.5)
            sleep(13)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)
        if flag != 1:
            self.media_ros.pub_vel(0.3, 0.0, -0.5)
            sleep(13)
            self.media_ros.pub_vel(0.0, 0.0, 0.0)

    def hand_threading(self, lmList, wrist):
        if self.event.is_set():
            self.event.clear()
            self.stop_status = 0
            self.index = 0
            fingers = self.hand_detector.fingersUp(lmList)
            gesture = self.hand_detector.get_gesture(lmList)
            print("fingers: ", fingers)
            print("gesture: ", gesture)
            if sum(fingers) == 5:
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
                if wrist != []:
                    print("Wrist: ", wrist[0][1], wrist[0][2])
                    self.follow_hand(wrist[0][1], wrist[0][2])
                print("Following hand")
                self.publish_status("Following hand")
                # sleep(0.5)

            elif sum(fingers) == 1 and fingers[1] == 1:
                self.media_ros.pub_vel(0.3, 0.0, 0.0)
                print("forward")
                self.publish_status("forward")
                sleep(0.5)

            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1:
                self.media_ros.pub_vel(-0.3, 0.0, 0.0)
                print("backward")
                self.publish_status("backward")
                sleep(0.5)

            elif (
                sum(fingers) == 3
                and fingers[1] == 1
                and fingers[2] == 1
                and fingers[3] == 1
            ):
                self.media_ros.pub_vel(0.3, 0.0, 0.5)
                print("left")
                self.publish_status("left")
                sleep(0.5)

            elif (
                sum(fingers) == 4
                and fingers[1] == 1
                and fingers[2] == 1
                and fingers[3] == 1
                and fingers[4] == 1
            ):
                self.media_ros.pub_vel(0.0, 0.0, -0.5)
                print("right")
                self.publish_status("right")
                sleep(0.5)

            elif (
                fingers[1] == fingers[4] == 1 and fingers[0] == 1 and sum(fingers) == 3
            ):
                print("s")
                self.publish_status("s")
                self.go_s()
                sleep(0.5)

            elif gesture == "Thumb_up":
                print("go_quadrilateral")
                self.publish_status("go_quadrilateral")
                self.go_quadrilateral()
                sleep(0.5)

            elif gesture == "OK":
                print("Go_circle")
                self.publish_status("Go_circle")
                self.Go_circle(1)
                sleep(1.0)

            elif gesture == "Thumb_down":
                print("Thumb_down")
                self.publish_status("Thumb_down")
                self.media_ros.pub_vel(0.3, 0.0, 0.0)
                sleep(2)
                self.media_ros.pub_vel(-0.3, 0.0, 0.0)
                sleep(2)
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
                sleep(1)
            else:
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
                self.publish_status("Idle")

            self.event.set()


def main():
    rclpy.init()
    pose_ctrl_arm = PoseCtrlArm("posectrlarm")
    # capture = cv.VideoCapture(0)
    # # capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    # capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    # while capture.isOpened():
    #     ret, frame = capture.read()
    #     frame = pose_ctrl_arm.process(frame)
    #     if cv.waitKey(1) & 0xFF == ord("q"):
    #         break
    #     #cv.imshow("frame", frame)
    # capture.release()
    # cv.destroyAllWindows()
    print("start pose_ctrl_arm node")
    rclpy.spin(pose_ctrl_arm)
