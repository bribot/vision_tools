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

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support CompressedImage in python
from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class recast:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("output/recast",
            Image)
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera/output",
            Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        try:
            img = self.bridge.imgmsg_to_cv2(ros_data,'bgr8')
        except CvBridgeError as e:
            print(e)
        
        ##Recast
        cv2.imshow('cv_img', img)
        cv2.waitKey(2)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img,'bgr8'))
        except CvBridgeError as e:
            print(e)

        #### Create CompressedIamge ####
        #msg = CompressedImage()
        #msg.header.stamp = rospy.Time.now()
        #msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        #self.image_pub.publish(msg)
        
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('recast', anonymous=True)    
    ic = recast()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image recast"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
