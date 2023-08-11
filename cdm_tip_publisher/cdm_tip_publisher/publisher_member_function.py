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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cdm_tip_msgs.msg import Resistance

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
        self.publisher_ = self.create_publisher(Resistance, 'r_sensor', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # init color stream
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # start streaming
        self.pipeline.start(config)



    def timer_callback(self):
        init = True


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
        if init:
            cv2.imshow('Robot Tip Locator', img_gray)
            init = False
        if none_zero_points is not None:
            coordinates_text = f"x={int(x)}, y={int(y)}"
            cv2.circle(img_gray, (int(x), int(y)), 10, (255, 255, 255), 1)
            cv2.putText(img_gray, coordinates_text, (int(x) - 0, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow('Robot Tip Locator', img_gray)
        if cv2.waitKey(1) & 0xFF == 27:
            self.pipeline.stop()
            cv2.destroyAllWindows()


        msg = Resistance()
        msg.pos1 = int(x)
        msg.pos2 = int(y)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing X: "%i"' % msg.pos1)
        self.get_logger().info('Publishing Y: "%i"' % msg.pos2)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = posPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
