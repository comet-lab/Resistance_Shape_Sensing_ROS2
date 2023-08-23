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
from rclpy.node import Node

from std_msgs.msg import String
from cdm_tip_msgs.msg import Resistance
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def write_value(X, Y, R):
    filename = 'sensor_aug23_3.csv'
    path = '/home/wenpeng/Documents/ros2_ws/src/CDM_Resistance_Shape_Sensing_ROS2/data'
    file_path = os.path.join(path, filename)
    data = [X, Y, R]
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow(data)
        print('value done')




class posSubscriber(Node):

    def __init__(self):
        super().__init__('posSubscriber')
        self.subscription = self.create_subscription(
            Resistance,
            '/r_sensor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.image_subscription = self.create_subscription(
            Image,
            'wrist_frame',
            self.image_callback,
            10)
        self.image_subscription
        self.br = CvBridge()

    def listener_callback(self, msg):
        Xpos = msg.pos1  # read value from ROS MSG
        Ypos = msg.pos2
        R = msg.resistance
        print(Xpos, Ypos, R)

        # write_value(Xpos, Ypos, R)

    def image_callback(self, img_msg):

        # Convert the ROS Image message to OpenCV format
        cv_img = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imshow('Frame Stream', cv_img)
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
