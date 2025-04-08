import cv2
from cv_bridge import CvBridge

class PreProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

    def process_color(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def process_depth(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")