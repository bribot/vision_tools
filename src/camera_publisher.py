#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
c=cv2.VideoCapture(1)

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

def cam_grabber():
   image_pub = rospy.Publisher("/image_raw",
            Image,queue_size=10)
   rospy.init_node('cam_grabber', anonymous=True)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
      ret, frame=c.read()
      #cv2.imshow("live feed",frame)
      #if cv2.waitKey(1)&0xFF==ord('q'):
      #   break
      image_np=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      msg = Image()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = "frame"
      msg.height = len(frame)
      msg.width = len(frame[1])
      msg.encoding="mono8"
      msg.step=640
      msg.data = image_np.tostring()
      # Publish new image
      image_pub.publish(msg)
      rate.sleep()
   #cv2.destroyAllWindows()

if __name__ == '__main__':
   try:
      cam_grabber()
   except rospy.ROSInterruptException:
      pass
