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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

try:
    import image_publisher.image_publisher_params as PARAMS
except ModuleNotFoundError:
    import image_publisher_params as PARAMS

    DEBUG = True


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")

        PARAMS.declare_params(self)
        self.use_camera = self.get_parameter(PARAMS.USE_CAMERA).value
        self.camera_input = self.get_parameter(PARAMS.CAMERA_INPUT).value
        self.output_name = self.get_parameter(PARAMS.OUTPUT_NAME).value
        self.image_path = self.get_parameter(PARAMS.IMAGE_PATH).value
        self.dev_mode = self.get_parameter(PARAMS.DEV_MODE).value
        self.frame_rate = self.get_parameter(PARAMS.FRAME_RATE).value

        self.publisher_ = self.create_publisher(Image, self.output_name, 10)
        timer_period = 1 / self.frame_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        if self.use_camera:
            self.cap = cv2.VideoCapture(self.camera_input)
            if not (self.cap.isOpened()):
                self.get_logger().error("Could not open camera")
                all_camera_idx_available = []

                for camera_idx in range(10):
                    cap = cv2.VideoCapture(camera_idx)
                    if cap.isOpened():
                        self.get_logger().info('Camera index available "%d"' % camera_idx)
                        all_camera_idx_available.append(camera_idx)
                        cap.release()

        self.bridge = CvBridge()

    def timer_callback(self):
        if self.use_camera:
            ret, frame = self.cap.read()
            if ret:
                frame = frame.astype(np.uint8)
                if self.dev_mode:
                    cv2.imshow("IMAGE", frame)
                    cv2.waitKey(1)
            self.pub_img(frame)
        else:
            self.load_and_pub_img(self.image_path)

        # self.get_logger().info('Publishing: "%d"' % self.i)
        self.i += 1

    def load_and_pub_img(self, img):
        # msg = Image()
        raw_img = cv2.imread(img)

        # ben_img = self.ben_load()
        # self.debug_image(ben_img, "ben")#

        # mah_img = self.mah_load(img)
        self.debug_image(raw_img, "img")

        # raw_img = cv2.imread(self.image_path)
        # pad_img = np.pad(mah_img, ((10, 10), (10, 10), (0, 0)))

        self.pub_img(raw_img)

    def pub_img(self, img_to_pub):

        msg = self.bridge.cv2_to_imgmsg(np.array(img_to_pub), "rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

    def iterate_on_dir(self, directory, methodToRun):
        # https://www.geeksforgeeks.org/how-to-iterate-over-files-in-directory-using-python/

        for filename in os.scandir(directory):
            if filename.is_file():
                self.get_logger().info('running function on "%s" ' % filename.path)
                methodToRun(filename.path)

    # def ben_load(self, img):
    #     # Read image
    #     raw_img = imageio.imread(img)

    #     # img = imageio.imread('path/to/your/image.png')
    #     # img = cv2.resize(img, (128, 128))

    #     # Image channels are expected to be RGB (not BGR)
    #     # If using OpenCV, make sure to flip the channel dimension
    #     # img = img[:, :, ::-1]

    #     # Pad image to convert size to 320x320
    #     img_to_return = np.pad(raw_img, ((10, 10), (10, 10), (0, 0)))

    #     # Bring the channel dimension to the front
    #     # Changes shape from (320, 320, 3) to (3, 320, 320)
    #     img = img_to_return.transpose(-1, 0, 1)
    #     return img_to_return

    def mah_load(self, img):
        # from https://answers.ros.org/question/359029/how-to-publish-batch-of-images-in-a-python-node-with-ros-2/
        # self.cv_image = cv2.imread('test.jpg') ### an RGB image
        cv_image = cv2.imread(img)  # an RGB image
        # self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))
        return cv_image

    def debug_image(self, img, name=""):
        # from https://www.tutorialkart.com/opencv/python/opencv-python-get-image-size/

        # get dimensions of image
        dimensions = img.shape

        # height, width, number of channels in image
        height = img.shape[0]
        width = img.shape[1]
        channels = img.shape[2]

        self.get_logger().info('Image Name         : "%s"' % name)
        self.get_logger().info('Image Dimension    : "%s"' % (dimensions,))
        self.get_logger().info('Image Height       : "%d"' % height)
        self.get_logger().info('Image Width        : "%d"' % width)
        self.get_logger().info('Number of Channels : "%d"' % channels)


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
