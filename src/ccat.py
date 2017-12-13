# -*- coding: utf-8 -*-
#"""
#Created on Wed Dec 13 11:55:21 2017

#author: bri
#"""
#!/usr/bin/env python
import sys,time
import cv2
import numpy as np
#Importing ROS libraries
import roslib
import rospy
from cv_bridge import CvBridge,CvBridgeError

#ROS messages
from sensor_msgs.msg import Image

VERBOSE=False

#Image Global Variables
minH = 22
maxH = 37
minS = 109
maxS = 255
minV = 115
maxV = 255
alpha=1.0
win1 = 'Trackbars'
win2 = 'Final'
win3 = 'Debug'

class ccat:
    
    def __init__(self):
        self.bridge = CvBridge()
        cv2.namedWindow(win1,0)
        #cv2.resizeWindow(win1,700,50)
        cv2.namedWindow(win2)
        cv2.setMouseCallback(win2,self.clickr)
        self.hsv=[0,0,0]
        #Region Of Interest
        self.roiPnt = [(0,0), (10,10)]
        self.roiSet = False
        self.roiDrag = False
        self.createTrackbars()
        self.subscriber = rospy.Subscriber("camera/output",Image,self.callback,queue_size=1)
        if VERBOSE:
            print("subscribed to camera/ouput")
            
    def callback(self,ros_data):
        
        #print("so far so good")
        #Transforming ROS msg to CV image
        try:
            img0 = self.bridge.imgmsg_to_cv2(ros_data,'bgr8')
        except CvBridgeError as e:
            print(e)
            
        if VERBOSE:
            cv2.imshow(win3,img0)
            cv2.waitKey(2)
        #Temporal HSV varivables initialization
        img = cv2.cvtColor(img0, cv2.COLOR_BGR2HSV)
        maxH = cv2.getTrackbarPos('Max H', win1)
        minH = cv2.getTrackbarPos('Min H', win1)
        maxS = cv2.getTrackbarPos('Max S', win1)
        minS = cv2.getTrackbarPos('Min S', win1)
        maxV = cv2.getTrackbarPos('Max V', win1)
        minV = cv2.getTrackbarPos('Min V', win1)
        alpha = cv2.getTrackbarPos("alpha", win1) * 0.01
        beta = 1.0
        #Arrays fortrackbars
        hsvMax = np.array((maxH, maxS, maxV))
        hsvMin = np.array((minH, minS, minV))
        #Image processing
        img_hsv = cv2.GaussianBlur(img,(11,11),0)
        img = cv2.inRange(img_hsv, hsvMin, hsvMax)
        img = self.noiseCleaner(img)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        img2 = cv2.addWeighted(img0, alpha, img, beta, 0.0)
        if self.roiSet | self.roiDrag:
            cv2.rectangle(img2, self.roiPnt[0], self.roiPnt[1], (0,255,0), 2)
        #Text on image
        tSize=0.4
        cv2.putText(img2,'H = %i, %i (%i)' %(minH,maxH,self.hsv[0]), (10,20),cv2.FONT_HERSHEY_SIMPLEX, tSize, (255,255,255))
        cv2.putText(img2,'S = %i, %i (%i)'%(minS,maxS,self.hsv[1]), (10,35),cv2.FONT_HERSHEY_SIMPLEX, tSize, (255,255,255))
        cv2.putText(img2,'V = %i, %i (%i)'%(minV,maxV,self.hsv[2]), (10,50),cv2.FONT_HERSHEY_SIMPLEX, tSize, (255,255,255))
        #Show Final Image
        cv2.imshow(win2,img2)
        cv2.waitKey(2)
        #ROI behaviour
        if self.roiSet:
            tmPnt=np.sort(self.roiPnt,axis=0)
            self.hsv=self.setHSV(img_hsv[tmPnt[0][1]:tmPnt[1][1],tmPnt[0][0]:tmPnt[1][0]])
            #cv2.imshow("test",img_hsv[roiPnt[0][1]:roiPnt[1][1],roiPnt[0][0]:roiPnt[1][0]])
            print self.hsv
            var = 15
            if self.hsv[0] < var:
                var = 5
            cv2.setTrackbarPos('Max H', win1, np.uint8(self.hsv[0])+var)
            cv2.setTrackbarPos('Min H', win1, np.uint8(self.hsv[0]-var))
            #cv2.setTrackbarPos('Max S', win1, int(hsv[1]+5))
            cv2.setTrackbarPos('Min S', win1, np.uint8(self.hsv[1]-15))
            #cv2.setTrackbarPos('Max V', win1, int(hsv[2]+15))
            cv2.setTrackbarPos('Min V', win1, np.uint8(self.hsv[2]-15))
            self.roiSet = False
        
    def clickr(self,event, x, y, flags, img):
        #global roiSet, roiDrag
        if (event == cv2.EVENT_MOUSEMOVE) & self.roiDrag:
            self.roiPnt[1] = (x,y)
        #print "what a drag"
        if (event == cv2.EVENT_LBUTTONDOWN) & (not(self.roiSet)):
            self.roiPnt[0]=(x,y)
            self.roiPnt[1] = (x,y)
            self.roiDrag = True
            print(self.roiPnt)
        elif (event == cv2.EVENT_LBUTTONUP) & (not(self.roiSet)):
            if (x,y) == self.roiPnt[0]:
                self.roiPnt[1]=(x+1,y+1)
            else:
                self.roiPnt[1]=(x,y)
            print(self.roiPnt)
            self.roiDrag = False
            self.roiSet=True
    
    def createTrackbars(self):
        cv2.createTrackbar('Max H',win1,maxH,255,self.nothing)
        cv2.createTrackbar('Min H',win1,minH,255,self.nothing)
        cv2.createTrackbar('Max S',win1,maxS,255,self.nothing)
        cv2.createTrackbar('Min S',win1,minS,255,self.nothing)
        cv2.createTrackbar('Max V',win1,maxV,255,self.nothing)
        cv2.createTrackbar('Min V',win1,minV,255,self.nothing)
        cv2.createTrackbar("alpha",win1,int(alpha*100),100,self.nothing)
        
    def setHSV(self,img):
    	hsv=[]
    	img= cv2.split(img)
    	hsv.append(np.mean(img[0]))
    	hsv.append(np.mean(img[1]))
    	hsv.append(np.mean(img[2]))
    	return hsv
    
    
    def noiseCleaner(self,img):
    	disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    	img = cv2.erode(img, disk, iterations = 3)
    	img = cv2.dilate(img, disk, iterations = 3)
    
    	img1 = cv2.erode(img,disk,iterations = 1)
    	img1 = cv2.bitwise_not(img1)
    	img = cv2.bitwise_and(img,img1)
    	return img
    
    def nothing(self,i):
        pass
        
        

def main(args):
    print "ok"
    rospy.init_node("ccat", anonymous=True)
    ic = ccat()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down Capture Color Assistant Tool')
    cv2.destroyAllWindows()
    

if __name__=="__main__":
    main(sys.argv)