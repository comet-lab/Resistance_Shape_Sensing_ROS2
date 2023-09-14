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

import numpy as np
import cv2
import pyrealsense2 as rs
import serial
import csv
# from usb_rs import Usb_rs
import threading
import time
import tkinter.messagebox
import matplotlib.pyplot as plt
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cdm_tip_msgs.msg import Resistance
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

global data_R
cali_width, cali_height = 170, 105
first_i, first_j = 100, 100
scale = 2
cali_pts = []
wrist_pts = []
M = []
wrist = 2


## Resistance Meter Package
class Usb_rs:

    def __init__(self, gui=False):
        self.ser = serial
        self.gui = gui
    
    #Open port
    def open(self, port, speed):
        ret = False

        try:
            self.ser = serial.Serial(port, speed,timeout=0)
            ret = True
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Open Error", e)
            else:
                print("Open error")
                print(e)
        
        return ret

    #Close port
    def close(self):
        ret = False

        try:   
            self.ser.close()
            ret = True
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Close Error", e)
            else:
                print("Close error")
                print(e)
        
        return ret
    #Send command
    def sendMsg(self, strMsg):
        ret = False

        try:
            strMsg = strMsg + '\r\n'                #Add a terminator, CR+LF, to transmitted command
            self.ser.write(bytes(strMsg, 'utf-8'))  #Convert to byte type and send
            ret = True
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Send Error", e)
            else:
                print("Send Error")
                print(e)

        return ret
    
    #Receive
    def receiveMsg(self, timeout):

        msgBuf = bytes(range(0))                    #Received Data

        try:
            start = time.time()                     #Record time for timeout
            while True:
                if self.ser.inWaiting() > 0:        #Is exist the data in the receive buffer?
                    rcv = self.ser.read(1)          #Receive 1 byte
                    if rcv == b"\n":                #End the loop when LF is received
                        msgBuf = msgBuf.decode('utf-8')
                        break
                    elif rcv == b"\r":              #Ignore the terminator CR
                        pass
                    else:
                        msgBuf = msgBuf + rcv
                
                #Timeout processing
                if  time.time() - start > timeout:
                    msgBuf = "Timeout Error"
                    break
        except Exception as e:
            if self.gui == True:
                tkinter.messagebox.showerror("Receive Error", e)
            else:
                print("Receive Error")
                print(e)
            msgBuf = "Error"

        return msgBuf
    
    #Transmit and receive commands
    def SendQueryMsg(self, strMsg, timeout):
        ret = Usb_rs.sendMsg(self, strMsg)
        if ret:
            msgBuf_str = Usb_rs.receiveMsg(self, timeout)   #Receive response when command transmission is succeeded
        else:
            msgBuf_str = "Error"

        return msgBuf_str

#Timeout(1sec)
Timeout_default = 1

## Resistance Reading Function
def read_R():
    #Instantiation of the usb_rs communication class
    serial1 = Usb_rs()

    #Connect
    # print("Port?")
    port = "/dev/ttyACM0"  
    # print("Speed?")
    speed = 9600
    if not serial1.open(port,speed):
        return
    
    #Send and receive commands

        # print("Please enter the command (Exit with no input)")
    command = "MEAS:RES?"
        #Exit if no input
    # if command == "":
    #     break
        #If the command contains "?"
    if "?" in command :
        msgBuf = serial1.SendQueryMsg(command, Timeout_default)
        #print(msgBuf) 
        #Send only
    else:
        serial1.sendMsg(command)
        
    serial1.close()
    return msgBuf

## Color Filter Tool Functions
def colorFilter(img_color, mode = "red"):
    # if mode == 'blue':
    #     lower = np.array([90, 50, 50])
    #     upper = np.array([150, 255, 255])
    # elif mode == 'green':
    #     lower = np.array([25, 120, 70])
    #     upper = np.array([95, 255, 255])
    # elif mode == 'red':
    #     lower_red1 = np.array([0, 100, 100])
    #     upper_red1 = np.array([10, 255, 255])

    #     lower_red2 = np.array([160, 100, 100])
    #     upper_red2 = np.array([180, 255, 255])
    # else:
    #     print("Please choose red, green, and blue mode")
    #     return
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)
    # mask_filter = cv2.inRange(hsv, lower, upper)
    # img_mask = cv2.bitwise_and(hsv, hsv, mask=mask_filter)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    img_mask = mask1 + mask2
    # img_gray = cv2.cvtColor(img_mask, cv2.COLOR_RGB2GRAY)
    result = cv2.bitwise_and(img_color, img_color, mask=img_mask)
    # img_thinned = cv2.ximgproc.thinning(img_gray)
    return result

def dot_locator(gray_image):
    # Find the non-zero point (tip) in the img
    non_zero_points = cv2.findNonZero(gray_image)

    if non_zero_points is not None:
        # Obtain tip location in the frame
        avg_pos = np.mean(non_zero_points, axis=0)
        x, y = avg_pos[0]
        # print(f"Position of the red point: (x={int(x)}, y={int(y)})")
    else:
        x = 0
        y = 0
        # print("No red point found in the image")
    return non_zero_points, x, y


def is_near_center(x, y, centers):
    for cx, cy in centers:
        distance = np.sqrt((cx - x)**2 + (cy - y)**2)
        if distance < 20:
            return (cx, cy)
    return None

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(cali_pts) < 4:
            cali_pts.append((x, y))
        else:
            if len(wrist_pts) < 3:
                wrist_pts.append((x, y))
        print(x,y)

def compute_angle(T, F, A):
    # Construct vectors
    v1 = np.subtract(F, T)
    v2 = np.subtract(F, A)
    
    # Dot product
    dot_product = np.dot(v1, v2)
    
    # Magnitudes of the vectors
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    
    # Angle in radians
    theta = np.arccos(dot_product / (magnitude_v1 * magnitude_v2))
    
    # Convert angle to degrees
    angle_degrees = np.degrees(theta)
    
    return angle_degrees

def extend_line(point1, point2, s):
    direction = np.array(point2) - np.array(point1)
    unit_direction = direction / np.linalg.norm(direction)

    # Calculate new extended endpoints
    new_point1 = point1 - s * unit_direction
    new_point2 = point2 + s * unit_direction
    
    return tuple(map(int, new_point1)), tuple(map(int, new_point2))


class posPublisher(Node):

    def __init__(self):
        super().__init__('posPublisher')
        self.init = True
        self.img_init = True
        self.wrist_init = True
        self.jpg = 0
        self.jpg_counter = 0
        self.publisher_ = self.create_publisher(Resistance, 'r_sensor', 10)
        self.image_publisher = self.create_publisher(Image, 'wrist_frame', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.br = CvBridge()
        self.centers_init = []


        # init realsense color stream
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # start streaming
        self.pipeline.start(config)
            

    def timer_callback(self):
        while (self.img_init):
            frames = self.pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            print("Prespective Transformation Init")
            img_color = np.asanyarray(color_frame.get_data())
            cv2.namedWindow("manual_calibration")
            cv2.setMouseCallback("manual_calibration", click_event)
            cv2.imshow("manual_calibration", img_color)
            cv2.waitKey(0)
            if (len(cali_pts) == 4):
                cv2.destroyAllWindows()
                # self.correction_M()
                self.height, self.width = img_color.shape[:2]
                # dst_pts = np.array([[first_i                 , first_j], 
                #            [first_i+scale*cali_width, first_j], 
                #            [first_i+scale*cali_width, first_j+scale*cali_height],
                #            [first_i                 , first_j+scale*cali_height] ], dtype="float32")
                # dst_pts = np.array([
                #     [0, 0],
                #     [self.width - 1, 0],
                #     [self.width - 1, self.height - 1],
                #     [0, self.height - 1]
                # ], dtype='float32')
                i, j = 100, 100
                dst_pts = np.array([
                    [0+i, 0+j],
                    [400+i, 0+j],
                    [400+i, 300+j],
                    [0+i, 300+j]
                ], dtype='float32')
            
                self.M = cv2.getPerspectiveTransform(np.array(cali_pts, dtype='float32'), dst_pts)
                self.img_init = False
                # print(M)
            img_warpped = cv2.warpPerspective(img_color, self.M, (self.width, self.height))
            print('Pixel to Real Scaling Init')
            cv2.namedWindow("scaling_finder")
            cv2.setMouseCallback("scaling_finder", click_event)
            cv2.imshow("scaling_finder", img_warpped)
            cv2.waitKey(0)
            if (len(wrist_pts) == 3):
                cv2.destroyAllWindows()
                distance_pixel = np.sqrt((wrist_pts[1][0] - wrist_pts[0][0])**2 + (wrist_pts[1][1] - wrist_pts[0][1])**2)
                # txt_x = int(0.5*(wrist_pts[-1][0] + wrist_pts[-2][0]) - 50)
                # txt_y = int(0.5*(wrist_pts[-1][1] + wrist_pts[-2][1]))
                self.p2r_scale = wrist/distance_pixel
                print(f"Current Pixel Distance is: {distance_pixel}")
                print(f"Pixel to Real Scaling is: {self.p2r_scale}")

        # wait for realsense color frame
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        # frame to np array as image
        color_img1 = np.asanyarray(color_frame.get_data())
        self.time0 = time.time() # pre processing time
        # color_img = cv2.warpPerspective(color_img1, self.M, (self.width, self.height))
        color_img = color_img1
        # apply color filter
        filtered_img = colorFilter(color_img)
        # convert to grayscale pic
        img_gray = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        none_zero_points, x, y = dot_locator(img_gray)
        bend_angle = -(compute_angle((int(x), int(y)), wrist_pts[1], wrist_pts[2]) - 180)
        # cv2.imshow('Color Frame', img_gray)
        # cv2.imshow('Position of Red Point', img_gray)
        # if self.init:
        #     # init the filter viewer
        #     cv2.imshow('Robot Tip Locator', img_gray)
        #     # img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2RGB)
        #     # plt.imshow(img_gray)
        #     # plt.show()
        #     # self.path = '/home/wenpeng/Documents/ros2_ws/src/CDM_Resistance_Shape_Sensing_ROS2/wrist_poses'
        #     self.init = False
        #     print('init done')

        ## tip viewer (comment out for fastest running speed)
        if none_zero_points is not None:
            # draw the tip in viewer
            coordinates_text = f"x={int(x)}, y={int(y)}, theta={bend_angle}"
            cv2.circle(img_gray, (int(x), int(y)), 10, (255, 255, 255), 1)
            cv2.putText(img_gray, coordinates_text, (int(x) - 0, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            wrist1, wrist_base = extend_line(wrist_pts[1], wrist_pts[2], 200)
            cv2.line(img_gray, wrist1, wrist_base,(255, 0, 0), 1)
            cv2.line(img_gray, (int(x), int(y)), wrist_pts[1], (255, 255, 255), 1)
            cv2.imshow('Robot Tip Locator', img_gray) # update viewer
            # data = read_R()
            # print(data_R)
        if cv2.waitKey(1) & 0xFF == 27: # viewer stop commands
            self.pipeline.stop()
            cv2.destroyAllWindows()

        # publish msg and Image
        msg = Resistance()
        msg.pos1 = int(x)
        msg.pos2 = int(y)
        msg.angle = bend_angle
        msg.resistance = float(data_R)
        msg.timestamp = time.time()
        self.publisher_.publish(msg)
        self.image_publisher.publish(self.br.cv2_to_imgmsg(color_img, encoding='bgr8'))
        self.time1 = time.time() # post processing time
        print(self.time1 - self.time0) # processing time
        # self.get_logger().info('Publishing X: "%i"' % msg.pos1)
        # self.get_logger().info('Publishing Y: "%i"' % msg.pos2)
        # self.get_logger().info('Publishing R: "%f"' % msg.resistance)
        # self.get_logger().info('Publishing wrist frame')


def main(args=None):
    x = threading.Thread(target=thread_function) 
    x.start()
    print("thread started")
    rclpy.init(args=args)

    node = posPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


## Resistance value reading thread
def thread_function(): 
    global data_R
    while 1:
        # data_R = read_R() # data read from resistance meter
        data_R = 0 # test code without resistance meter
        # print(data_R)


if __name__ == '__main__':
    main()
