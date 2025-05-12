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


class LineDetect(Node):
    def __init__(self, name):
        super().__init__(name)
        # create a publisher
        self.pub_cmdVel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_Buzzer = self.create_publisher(UInt16, "/beep", 1)
        self.pub_Servo1 = self.create_publisher(Int32, "servo_s1", 10)
        self.pub_Servo2 = self.create_publisher(Int32, "servo_s2", 10)
        # create a subscriber
        # self.sub_JoyState = self.create_subscription(
        #     Bool, "/JoyState", self.JoyStateCallback, 1
        # )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.registerScan, 1
        )
        self.Joy_active = False
        self.img = None
        self.circle = ()
        self.hsv_range = ()
        self.Roi_init = ()
        self.warning = 1
        self.end = 0
        self.Start_state = True
        self.dyn_update = False
        self.Buzzer_state = False
        self.select_flags = False
        self.Track_state = "identify"
        self.windows_name = "frame"
        self.cols, self.rows = 0, 0
        self.Mouse_XY = []
        self.hsv_text = (
            "/root/yahboomcar_ws/src/yahboomcar_astra/yahboomcar_astra/colorHSV.text"
        )
        self.color = color_follow()
        self.scale = 1000
        self.FollowLinePID = (60, 0, 20)
        self.linear = 0.2
        self.LaserAngle = 30
        self.ResponseDist = 0.55
        self.PWMServo_X = 0
        self.PWMServo_Y = -75
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y
        self.PID_init()
        self.declare_param()
        self.img_flip = True
        self.refresh = False
        self.capture = None
        # if cv_edition[0]=='3': self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        # else: self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.socket = self.create_subscription(
            UInt16, "/listener", self.onSocketCallback, 1
        )
        self.timer = self.create_timer(0.001, self.on_timer)
        # self.media_ros = Media_ROS()
        self.pub_img = self.create_publisher(CompressedImage, "/image", 1)
        self.pub_binaryImg = self.create_publisher(CompressedImage, "/binary_image", 1)
        self.publisher_ = self.create_publisher(String, "/status", 1)
        self.publish_status("Idle")
        self.finalFrame = None
        self.finalBinary = None
        self.timer2 = self.create_timer(0.001, self.start_image_publisher_thread)
        self.timer3 = self.create_timer(0.001, self.start_binaryImage_publisher_thread)
        self.past_listener = 0
        # self.start_image_publisher_thread()
        # self.start_binaryImage_publisher_thread()

    def declare_param(self):
        # HSV
        self.declare_parameter("Hmin", 0)
        self.Hmin = self.get_parameter("Hmin").get_parameter_value().integer_value
        self.declare_parameter("Smin", 85)
        self.Smin = self.get_parameter("Smin").get_parameter_value().integer_value
        self.declare_parameter("Vmin", 126)
        self.Vmin = self.get_parameter("Vmin").get_parameter_value().integer_value
        self.declare_parameter("Hmax", 9)
        self.Hmax = self.get_parameter("Hmax").get_parameter_value().integer_value
        self.declare_parameter("Smax", 253)
        self.Smax = self.get_parameter("Smax").get_parameter_value().integer_value
        self.declare_parameter("Vmax", 253)
        self.Vmax = self.get_parameter("Vmax").get_parameter_value().integer_value
        # PID
        self.declare_parameter("Kp", 60)
        self.Kp = self.get_parameter("Kp").get_parameter_value().integer_value
        self.declare_parameter("Ki", 0)
        self.Ki = self.get_parameter("Ki").get_parameter_value().integer_value
        self.declare_parameter("Kd", 20)
        self.Kd = self.get_parameter("Kd").get_parameter_value().integer_value
        # other
        self.declare_parameter("scale", 1000)
        self.scale = self.get_parameter("scale").get_parameter_value().integer_value
        self.declare_parameter("LaserAngle", 30)
        self.LaserAngle = (
            self.get_parameter("LaserAngle").get_parameter_value().integer_value
        )
        self.declare_parameter("linear", 0.18)
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.declare_parameter("ResponseDist", 0.55)
        self.ResponseDist = (
            self.get_parameter("ResponseDist").get_parameter_value().double_value
        )
        self.declare_parameter("refresh", False)
        self.refresh = self.get_parameter("refresh").get_parameter_value().bool_value
        self.pub_Servo2.publish(self.s2_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)
        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo1.publish(self.s1_init_angle)
        # NOTE: Added
        self.declare_parameter("event", 0)
        self.event = self.get_parameter("event").get_parameter_value().integer_value
        self.declare_parameter("X", 0)
        self.declare_parameter("Y", 0)
        self.x = self.get_parameter("X").get_parameter_value().integer_value
        self.y = self.get_parameter("Y").get_parameter_value().integer_value

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0],
        )

    # def onMouse(self, event, x, y, flags, param):
    # 	if event == 1:
    # 		self.Track_state = 'init'
    # 		self.select_flags = True
    # 		self.Mouse_XY = (x,y)
    # 	if event == 4:
    # 		self.select_flags = False
    # 		self.Track_state = 'mouse'
    # 	if self.select_flags == True:
    # 		self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
    # 		self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
    # 		self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def publish_status(self, msg: str):
        # The status message
        status_msg = String()

        # You can customize the status text here
        status_msg.data = msg

        # Publish the message to the "/status" topic
        self.publisher_.publish(status_msg)

        # Log the status being published
        self.get_logger().info(f"Publishing: {status_msg.data}")

    def onSocketCallback(self, msg):
        if msg.data != 2:
            self.Joy_active = True
            if self.capture and self.capture.isOpened():
                self.pub_cmdVel.publish(Twist())
                self.capture.release()
                print("Follow_line Camera released.")
        else:
            self.Joy_active = False
            
        if self.past_listener != 2 and msg.data == 2:
            while(True):
                try:
                    self.capture = cv.VideoCapture(0)
                    self.pub_Servo2.publish(self.s2_init_angle)
                    self.pub_Servo2.publish(self.s2_init_angle)
                    self.pub_Servo1.publish(self.s1_init_angle)
                    self.pub_Servo1.publish(self.s1_init_angle)
                    if not self.capture.isOpened():
                        raise RuntimeError("Failed to open camera on index 0")

                    # self.capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
                    # self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
                    print("Follow_line Camera opened. FPS:", self.capture.get(cv.CAP_PROP_FPS))
                    break

                except Exception as e:
                    print(f"[ERROR] Follow_line onSocketCallback: {e}")
                    # Optionally: handle fallback behavior or retry
                    
        self.past_listener = msg.data

        self.get_param()
        if self.event == 0:
            self.cancel()
        elif self.event == 1:
            self.Reset()
        elif self.event == 2:
            self.publish_status("Init")
            self.Track_state = "init"
            self.select_flags = True
            if len(self.Mouse_XY) == 2:
                self.Mouse_XY[1] = [self.x, self.y]
            else:
                self.Mouse_XY.append([self.x, self.y])
                self.Mouse_XY.append([self.x + 5, self.y + 5])
        elif self.event == 3:
            self.publish_status("Tracking")
            self.Track_state = "tracking"
        if self.select_flags == True and len(self.Mouse_XY) == 2:
            self.cols = min(self.Mouse_XY[0][0], self.Mouse_XY[1][0]), min(
                self.Mouse_XY[0][1], self.Mouse_XY[1][1]
            )
            self.rows = max(self.Mouse_XY[0][0], self.Mouse_XY[1][0]), max(
                self.Mouse_XY[0][1], self.Mouse_XY[1][1]
            )
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
            # self.Mouse_XY = []

    def execute(self, point_x, color_radius):

        if self.Joy_active == True:
            if self.Start_state == True:
                self.PID_init()
                self.Start_state = False
            return
        self.Start_state = True
        if color_radius == 0:
            print("Not Found")
            self.publish_status("Not Found")
            self.pub_cmdVel.publish(Twist())
        else:
            twist = Twist()
            b = UInt16()
            [z_Pid, _] = self.PID_controller.update([(point_x - 320) * 1.0 / 16, 0])
            # [z_Pid, _] = self.PID_controller.update([(point_x - 10)*1.0/16, 0])
            if self.img_flip == True:
                twist.angular.z = -z_Pid  # -z_Pid
            # else: twist.angular.z = (twist.angular.z+z_Pid)*0.2
            else:
                twist.angular.z = +z_Pid
            # point_x = point_x
            # twist.angular.z=-(point_x-320)*1.0/128.0

            twist.linear.x = self.linear
            if self.warning > 10:
                print("Obstacles ahead !!!")
                self.publish_status("Obstacles")
                self.pub_cmdVel.publish(Twist())
                self.Buzzer_state = True
                b.data = 1
                self.pub_Buzzer.publish(b)
            else:
                if self.Buzzer_state == True:
                    b.data = 0
                    for i in range(3):
                        self.pub_Buzzer.publish(b)
                    self.Buzzer_state = False

                if abs(point_x - 320) < 40:
                    # if abs(point_x-30)>40:
                    twist.angular.z = 0.0
                if self.Joy_active == False:
                    self.publish_status("Tracking")
                    self.pub_cmdVel.publish(twist)
                else:
                    twist.angular.z = 0.0

    def process(self, rgb_img):

        binary = []
        rgb_img = cv.resize(rgb_img, (640, 480))

        if self.img_flip == True:
            rgb_img = cv.flip(rgb_img, 1)
        # if action == 32: self.Track_state = 'tracking'
        # elif action == ord('i') or action == 105: self.Track_state = "identify"
        # elif action == ord('r') or action == 114: self.Reset()
        # elif action == ord('q') or action == 113: self.cancel()
        if self.Track_state == "init":
            # cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            # cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            # TODO: Get pixel coords
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if (
                    self.Roi_init[0] != self.Roi_init[2]
                    and self.Roi_init[1] != self.Roi_init[3]
                ):
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.dyn_update = True
            else:
                self.Track_state = "init"
        elif self.Track_state == "identify":
            # print(self.circle[0])
            if os.path.exists(self.hsv_text):
                self.hsv_range = read_HSV(self.hsv_text)
            else:
                self.Track_state = "init"
        if self.Track_state != "init" and len(self.hsv_range) != 0:
            rgb_img, binary, self.circle = self.color.line_follow(
                rgb_img, self.hsv_range
            )
            if self.dyn_update == True:
                write_HSV(self.hsv_text, self.hsv_range)
                self.Hmin = rclpy.parameter.Parameter(
                    "Hmin", rclpy.Parameter.Type.INTEGER, self.hsv_range[0][0]
                )
                self.Smin = rclpy.parameter.Parameter(
                    "Smin", rclpy.Parameter.Type.INTEGER, self.hsv_range[0][1]
                )
                self.Vmin = rclpy.parameter.Parameter(
                    "Vmin", rclpy.Parameter.Type.INTEGER, self.hsv_range[0][2]
                )
                self.Hmax = rclpy.parameter.Parameter(
                    "Hmax", rclpy.Parameter.Type.INTEGER, self.hsv_range[1][0]
                )
                self.Smax = rclpy.parameter.Parameter(
                    "Smax", rclpy.Parameter.Type.INTEGER, self.hsv_range[1][1]
                )
                self.Vmax = rclpy.parameter.Parameter(
                    "Vmax", rclpy.Parameter.Type.INTEGER, self.hsv_range[1][2]
                )

                all_new_parameters = [
                    self.Hmin,
                    self.Smin,
                    self.Vmin,
                    self.Hmax,
                    self.Smax,
                    self.Vmax,
                ]
                self.set_parameters(all_new_parameters)
                self.dyn_update = False
        if self.Track_state == "tracking":
            if len(self.circle) != 0:
                threading.Thread(
                    target=self.execute, args=(self.circle[0], self.circle[2])
                ).start()
        else:
            if self.Start_state == True:
                # self.pub_cmdVel.publish(Twist())
                self.Start_state = False
        return rgb_img, binary

    def start_image_publisher_thread(self):
        thread = threading.Thread(target=self.image_publisher_loop, daemon=True)
        thread.start()

    def start_binaryImage_publisher_thread(self):
        thread = threading.Thread(target=self.binaryImage_publisher_loop, daemon=True)
        thread.start()

    def image_publisher_loop(self):
        if rclpy.ok():
            if self.pub_img.get_subscription_count() > 0 and self.Joy_active == False:
                # frame = self.get_latest_frame()  # Replace with actual frame retrieval logic
                if self.finalFrame is not None:
                    _, buffer = cv2.imencode(".jpg", self.finalFrame)
                    msg = CompressedImage()
                    msg.format = "jpeg"
                    msg.data = buffer.tobytes()
                    self.pub_img.publish(msg)
            # rclpy.spin_once(self, timeout_sec=0.01)  # Optional: keep the node alive if needed
            # time.sleep(0.03)  # ~30 FPS, adjust as needed

    def binaryImage_publisher_loop(self):
        if rclpy.ok():
            if self.pub_img.get_subscription_count() > 0 and self.Joy_active == False:
                # binary = self.get_latest_frame()  # Replace with actual frame retrieval logic
                if self.finalBinary is not None:
                    _, buffer = cv2.imencode(".jpg", self.finalBinary)
                    msg = CompressedImage()
                    msg.format = "jpeg"
                    msg.data = buffer.tobytes()
                    self.pub_binaryImg.publish(msg)
            # rclpy.spin_once(self, timeout_sec=0.01)  # Optional: keep the node alive if needed
            # time.sleep(0.03)  # ~30 FPS, adjust as needed

    def on_timer(self):
        # self.get_param()
        if self.capture is None or not self.capture.isOpened():
            return
        ret, self.frame = self.capture.read()
        if not ret:
            return
        # TODO: Get action
        action = cv.waitKey(10) & 0xFF
        self.frame, self.binary = self.process(self.frame)
        start = time.time()
        fps = 1 / (start - self.end)
        text = "FPS : " + str(int(fps))
        self.end = start
        cv.putText(
            self.frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1
        )
        self.finalFrame = self.frame
        self.finalBinary = self.binary
        # NOTE: Added
        if action == ord("q") or action == 113:
            self.capture.release()
            cv.destroyAllWindows()

    # def JoyStateCallback(self, msg):
    #     if not isinstance(msg, Bool):
    #         return
    #     self.Joy_active = msg.data

    def registerScan(self, scan_data):

        self.warning = 1
        if not isinstance(scan_data, LaserScan):
            return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        ranges = np.array(scan_data.ranges)
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to:
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
            if abs(angle) < 90:
                if ranges[i] > 0.2:
                    if ranges[i] < self.ResponseDist:
                        self.warning += 1

    def Reset(self):
        self.PID_init()
        self.Track_state = "init"
        self.hsv_range = ()
        self.Joy_active = False
        self.Mouse_XY = []
        self.Roi_init = ()
        self.cols = 0, 0
        self.rows = 0, 0
        self.pub_cmdVel.publish(Twist())
        print("Reset succes!!!")
        self.publish_status("Reset")
        # NOTE: Added
        self.select_flags = False

    def cancel(self):
        self.Mouse_XY = []
        self.select_flags = False
        self.Roi_init = ()
        self.cols = 0, 0
        self.rows = 0, 0

    def get_param(self):
        # hsv
        self.Hmin = self.get_parameter("Hmin").get_parameter_value().integer_value
        self.Smin = self.get_parameter("Smin").get_parameter_value().integer_value
        self.Vmin = self.get_parameter("Vmin").get_parameter_value().integer_value
        self.Hmax = self.get_parameter("Hmax").get_parameter_value().integer_value
        self.Smax = self.get_parameter("Smax").get_parameter_value().integer_value
        self.Vmax = self.get_parameter("Vmax").get_parameter_value().integer_value
        # kpi
        self.Kd = self.get_parameter("Kd").get_parameter_value().integer_value
        self.Ki = self.get_parameter("Ki").get_parameter_value().integer_value
        self.Kp = self.get_parameter("Kp").get_parameter_value().integer_value
        self.FollowLinePID = (self.Kp, self.Ki, self.Kd)
        #
        self.scale = self.get_parameter("scale").get_parameter_value().integer_value
        self.LaserAngle = (
            self.get_parameter("LaserAngle").get_parameter_value().integer_value
        )
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.ResponseDist = (
            self.get_parameter("ResponseDist").get_parameter_value().double_value
        )
        self.refresh = self.get_parameter("refresh").get_parameter_value().bool_value
        # NOTE: Added
        self.event = self.get_parameter("event").get_parameter_value().integer_value
        self.x = self.get_parameter("X").get_parameter_value().integer_value
        self.y = self.get_parameter("Y").get_parameter_value().integer_value


def main():
    rclpy.init()
    linedetect = LineDetect("follow_line")
    print("start follow_line node")
    rclpy.spin(linedetect)
