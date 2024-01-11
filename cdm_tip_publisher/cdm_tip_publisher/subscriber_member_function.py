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

import csv
import os
import rclpy
import cv2
import time
from rclpy.node import Node

from std_msgs.msg import String
from cdm_tip_msgs.msg import Resistance
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def write_value(angle, strain, R, timestamp):
    # csv name and file path (NEED CHANGE FOR DIFFERENT TRAILS)
    filename = 'CAAR_Carbon_5mm_5.csv'
    path = '/home/wenpeng/Documents/ros2_ws/src/CDM_Resistance_Shape_Sensing_ROS2/data/Jan11'
    file_path = os.path.join(path, filename)
    data = [angle, strain, R, timestamp]
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow(data)
        print('value done')




class posSubscriber(Node):

    def __init__(self):
        super().__init__('posSubscriber')
        # tip position & Resistance value subscriber
        self.subscription = self.create_subscription(
            Resistance,
            '/r_sensor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # wrist pose subscriber
        self.image_subscription = self.create_subscription(
            Image,
            'wrist_frame',
            self.image_callback,
            10)
        self.image_subscription  # prevent unused variable warning

        # Class shared variables init
        self.br = CvBridge()
        self.flag = False
        self.jpg = 1
        self.jpg_counter = 0
        self.ts0 = 0.0
        self.stamp = 0.0

        # FilePath for saving the wrist frames
        self.img_path = '/home/wenpeng/Documents/ros2_ws/src/CDM_Resistance_Shape_Sensing_ROS2/wrist_poses/trail1'

    def listener_callback(self, msg):
        # read value from ROS MSG
        bend_angle = msg.angle
        strain = msg.strain
        R = msg.resistance
        self.ts = msg.timestamp #
        if self.jpg == 1:
            self.ts0 = self.ts
        if (bend_angle): # check if camera is working
            self.flag = True # flag for saving wrist pose
        t = round(self.stamp+self.ts-self.ts0, 2)
        print(bend_angle, strain, R, t)
        write_value(bend_angle, strain, R, t) # write to data csv
        self.stamp += self.ts - self.ts0 # update stamp for next data
        self.ts0 = self.ts # updating last data time
        

    def image_callback(self, img_msg):

        # Convert the ROS Image message to OpenCV format
        self.cv_img = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        if self.flag:
            # print(self.jpg_counter)
            if self.jpg_counter == 0:  # wrist frame sample rate (Change counter++ and counter==)
                    plt_img = self.cv_img
                    # Comment here for testing
                    # cv2.imwrite(os.path.join(self.img_path, str(self.jpg)+'.jpg'), plt_img) # saving each wrist pose
                    self.jpg += 1
                    self.jpg_counter = 0
            self.jpg_counter += 0
            
        # Show Real-time frame stream in separate window
        cv2.imshow('Frame Stream', self.cv_img)  
        cv2.waitKey(1)
        


def main(args=None):
    rclpy.init(args=args)

    node = posSubscriber()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
