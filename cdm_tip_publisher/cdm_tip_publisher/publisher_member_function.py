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


class posPublisher(Node):

    def __init__(self):
        super().__init__('posPublisher')
        self.init = True
        self.jpg = 0
        self.jpg_counter = 0
        self.publisher_ = self.create_publisher(Resistance, 'r_sensor', 10)
        self.image_publisher = self.create_publisher(Image, 'wrist_frame', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.br = CvBridge()

        # init color stream
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # start streaming
        self.pipeline.start(config)



    def timer_callback(self):

        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        color_img = np.asanyarray(color_frame.get_data())

        # apply color filter
        filtered_img = colorFilter(color_img)
        # convert to grayscale pic
        img_gray = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        none_zero_points, x, y = dot_locator(img_gray)
        # cv2.imshow('Color Frame', img_gray)
        # cv2.imshow('Position of Red Point', img_gray)
        if self.init:
            cv2.imshow('Robot Tip Locator', img_gray)
            # img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2RGB)
            # plt.imshow(img_gray)
            # plt.show()
            # self.path = '/home/wenpeng/Documents/ros2_ws/src/CDM_Resistance_Shape_Sensing_ROS2/wrist_poses'
            self.init = False
            print('init done')
        if none_zero_points is not None:
            coordinates_text = f"x={int(x)}, y={int(y)}"
            cv2.circle(img_gray, (int(x), int(y)), 10, (255, 255, 255), 1)
            cv2.putText(img_gray, coordinates_text, (int(x) - 0, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            # self.jpg_counter += 1
            # print(self.jpg_counter)
            # if self.jpg_counter == 100:
            #     self.jpg_counter = 0
            #     # plt_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            #     cv2.imwrite(os.path.join(self.path, str(self.jpg)+'.jpg'), color_img)
            #     # plt.savefig(str(self.jpg)+'.png')
            #     print('saved pose')
            #     self.jpg += 1
            cv2.imshow('Robot Tip Locator', img_gray)
            # img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2RGB)
            # plt.imshow(img_gray)
            # plt.show()
            # data = read_R()
            # print(data_R)
        if cv2.waitKey(1) & 0xFF == 27:
            self.pipeline.stop()
            cv2.destroyAllWindows()


        msg = Resistance()
        msg.pos1 = int(x)
        msg.pos2 = int(y)
        msg.resistance = float(data_R)
        self.publisher_.publish(msg)
        self.image_publisher.publish(self.br.cv2_to_imgmsg(color_img, encoding='bgr8'))
        # self.get_logger().info('Publishing X: "%i"' % msg.pos1)
        # self.get_logger().info('Publishing Y: "%i"' % msg.pos2)
        # self.get_logger().info('Publishing R: "%f"' % msg.resistance)
        # self.get_logger().info('Publishing wrist frame')
        self.i += 1


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
        # data_R = read_R()
        data_R = 0
        # print(data_R)


if __name__ == '__main__':
    main()
