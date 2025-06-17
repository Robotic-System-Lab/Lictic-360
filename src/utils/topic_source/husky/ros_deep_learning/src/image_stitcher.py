#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageStitcher:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber("/camera1/image_raw", Image, self.image_callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/image_raw", Image, self.image_callback2)
        self.image_sub3 = rospy.Subscriber("/camera3/image_raw", Image, self.image_callback3)
        self.image_pub = rospy.Publisher("/combined_image", Image, queue_size=10)

        self.image1 = None
        self.image2 = None
        self.image3 = None

    def image_callback1(self, msg):
        self.image1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.stitch_images()

    def image_callback2(self, msg):
        self.image2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.stitch_images()

    def image_callback3(self, msg):
        self.image3 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.stitch_images()

    def stitch_images(self):
        if self.image1 is not None and self.image2 is not None and self.image3 is not None:
            # Gabungkan gambar secara horizontal
            stitched_image = np.hstack((self.image1, self.image2, self.image3))
            stitched_image_msg = self.bridge.cv2_to_imgmsg(stitched_image, "bgr8")
            self.image_pub.publish(stitched_image_msg)

if __name__ == '__main__':
    rospy.init_node('image_stitcher')
    stitcher = ImageStitcher()
    rospy.spin()
