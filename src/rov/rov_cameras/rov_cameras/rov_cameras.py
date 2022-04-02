import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

FPS = float(30.0)

class ROVCameras(Node):
    def __init__(self):
        super().__init__("rov_cameras")
        self.cameras = [cv2.VideoCapture(i) for i in range(0,4)]
        self.active_cameras = set(["cam0", "cam1", "cam2", "cam3"])
        self.br = CvBridge()

        self.camera_publishers = {
            "cam0":self.create_publisher(Image, "cam0_image", 10),
            "cam1":self.create_publisher(Image, "cam1_image", 10),
            "cam2":self.create_publisher(Image, "cam2_image", 10),
            "cam3":self.create_publisher(Image, "cam3_image", 10)
        }

        self.create_timer(1/FPS, self.cams)
    
    def cams(self):
        for camera in self.active_cameras:
            ret, frame = self.cameras[int(camera[-1])].read()
            if(ret):
                self.camera_publishers[camera].publish(self.br.cv2_to_imgmsg(frame))
        pass

    def toggleCam(self, cam:int):
        if(f"cam{cam}" in self.active_cameras):
            self.active_cameras.remove(f"cam{cam}")
        else:
            self.active_cameras.add(f"cam{cam}")


def main():
    rclpy.init()
    rclpy.spin(ROVCameras())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
