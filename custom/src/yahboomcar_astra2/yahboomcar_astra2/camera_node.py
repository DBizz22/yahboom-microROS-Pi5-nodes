# ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image

# common lib
import os
import threading
import math
from yahboomcar_astra.follow_common import *
from std_msgs.msg import Int32, Bool, UInt16

# from yahboomcar_astra2.media_library import Media_ROS
from yahboomcar_msgs.msg import *
import base64
import cv2


RAD2DEG = 180 / math.pi
print("import finish")
cv_edition = cv.__version__
print("cv_edition: ", cv_edition)


class CameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.capture = None
        self.pub_img = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", 1
        )
        self.socket = self.create_subscription(
            UInt16, "/listener", self.onSocketCallback, 1
        )
        self.timer = self.create_timer(0.05, self.on_timer)

    def onSocketCallback(self, msg):
        print("CameraNode onSocketCallback: ", msg.data)
        if msg.data != 0:
            if self.capture and self.capture.isOpened():
                self.capture.release()
                print("CameraNode Camera released.")

        if msg.data == 0:
            if self.capture is None or not self.capture.isOpened():
                print("CameraNode onSocketCallback: Opening camera...")
            while True:
                try:
                    self.capture = cv.VideoCapture(0)
                    if not self.capture.isOpened():
                        raise RuntimeError("Failed to open camera on index 0")
                    print("Camera opened. FPS:", self.capture.get(cv.CAP_PROP_FPS))
                    break
                except Exception as e:
                    print(f"[ERROR] CameraNode onSocketCallback: {e}")
                    # Optionally: handle fallback behavior or retry

    def on_timer(self):
        if self.capture and self.capture.isOpened():
            ret, frame = self.capture.read()
            if ret:
                self.process(frame)
            else:
                print("Failed to read frame from camera.")

    def process(self, frame):
        frame = cv.flip(frame, 1)
        _, buffer = cv2.imencode(".jpg", frame)
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        self.pub_img.publish(msg)
        return frame


def main():
    rclpy.init()
    cameraNode = CameraNode("CameraNode")
    rclpy.spin(cameraNode)
