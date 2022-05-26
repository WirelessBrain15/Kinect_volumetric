#!/usr/bin/env python3


# The code follows a basic approach of subscribing to the incoming rgb and depth camera
# and using image processing on the rgb data to check whether an object is in jone and 
# calculate the surface area of object and check the depth through depth data.


import sys
import rospy
import roslib
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import numpy.ma as ma
import message_filters
from time import time

from scipy import spatial
from imutils import perspective
from imutils import contours
import imutils
import cProfile
import re


class image_test:
    
    def __init__(self):

        self.t_old = time()
        self.depth_full = None
        self.base_height = 0.0
        self.new_height = 100
        self.area = 10000
        self.cyF = 0
        self.cxF = 0
        self.cx = 0
        self.cy = 0
        self.ppmA = None
        self.ppmB = None
        self.flag = False
        self.calibFlag1 = False
        self.calibFlag2 = False
        # self.paramCheck = False

        ## variable for static reference
        #self.pixelPerMetricA = None
        #self.pixelPerMetricB = None

        self.bridge = CvBridge()
    
    # def calibration(self):
        # array = np.empty([3])
        # array = np.asarray([0.6, 1.0, 0.4])
        # print(array)

        # self.calibFlag1 = True
        # self.calibFlag2 = True
        # array = np.load('parameters.npy')
        # print(array)
        # if array is None:
        #     array[0] = self.ppmA
        #     array[1] = self.ppmB
        #     array[2] = self.depth_full
        #     np.save('parameters.npy', array)
        #     print("None")
        # else:
        #     self.ppmA = array[0]
        #     self.ppmB = array[1]
        #     self.depth_full = array[2]
        #     print("Done")
        #     print(array)

    def calibration(self):
        self.calibFlag1 = True
        self.calibFlag2 = True
        rospy.spin()
        a = self.ppmA
        b = self.ppmB
        c = self.depth_full
        np.savez('parameters.npy', a=a, b=b, c=c)
        print("ppmA : ", a)
        print("ppmB : ", b)
        print("Base height array saved! ")
        print("calibration done . . . ")
    
    def readCalibration(self):
        # array = np.asarray([0.6, 1.0, 0.4])
        # print(array)

        array = np.load('parameters.npy')
        if array is None:
            print("Parameters missing")
            self.calibration()
        else:
            self.ppmA = array['a']
            self.ppmB = array['b']
            self.depth_full = array['c']
            # self.paramCheck = True 
            print("Parameters found! ")

    def depthListener(self):
        self.image_depth = rospy.Subscriber("/camera/depth/image", Image, image_test().depthCallback)
        rospy.spin()
    def rgbListener(self):
        self.image_rgb = rospy.Subscriber("/camera/rgb/image_color", Image, image_test().rgbCallback)
        image_test().depthListener()
        # rospy.spin()

    def midpoint(self, ptA, ptB):

        # Function to calculate the mid point between ptA and ptB

	    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

    #def boxMark(self, cnt, orig, pixelPerMetricA, pixelPerMetricB): # moving reference
    def boxMark(self, cnt, orig): ## static reference

        # Function to mark the object and calculate the pixels per meter constant

        #time_now = time()
        box = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)
                
        # Calculate the mid points of all sides and join them
        (tl, tr, br, bl) = box
        tltrX, tltrY = image_test().midpoint(tl, tr)
        blbrX, blbrY = image_test().midpoint(bl, br)
        tlblX, tlblY = image_test().midpoint(tl, bl)
        trbrX, trbrY = image_test().midpoint(tr, br)
                
        cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)

        ## Draw lines between the midpoints
        cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
        (255, 0, 255), 2)
        cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
        (255, 0, 255), 2)
        ## compute the Euclidean distance between the midpoints
        dA = spatial.distance.euclidean((tltrX, tltrY), (blbrX, blbrY))
        dB = spatial.distance.euclidean((tlblX, tlblY), (trbrX, trbrY))
        
        # pixelPerMetric calculation

        #if pixelPerMetricA is None and pixelPerMetricB is None: # moving reference
        #    pixelPerMetricA = dA/0.06
        #    pixelPerMetricB = dB/0.06
        #dimA = dA/pixelPerMetricA
        #dimB = dB/pixelPerMetricB

        if self.ppmA is None and self.ppmB is None: # static reference
            self.ppmA = dA/0.06   # 6cm x 6cm reference
            self.ppmB = dB/0.06
        dimA = dA/self.ppmA
        dimB = dB/self.ppmB
        
        ## draw the object sizes on the image
        cv2.putText(orig, "{:.3f}m".format(dimA),
        (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2)
        cv2.putText(orig, "{:.3f}m".format(dimB),
        (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2)
        #print time() - time_now
        
        #return pixelPerMetricA, pixelPerMetricB, dimA*dimB # moving reference
        return dimA*dimB #static reference

    def depthCallback(self, depth):
        # Function to process the depth data

        ## Image to cv_image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
        except CvBridgeError:
            rospy.loginfo("depth failed")
        
        ## cv_image to array
        if self.depth_full is None: # kinect height
            self.depth_full = np.array(depth_image, dtype = np.dtype('float32'))
            print("Init done")

        # Process data only when object is near center point
        if self.flag == True:
            temp = np.zeros((10,10), dtype=np.dtype('float32'))
            self.depth_new = np.array(depth_image, dtype = np.dtype('float32'))
            temp = self.depth_new[self.cx-5:self.cx+5,self.cy-5:self.cy+5]
                    
            # Ensure that the max height of object is taken (least distance from ground)
            if self.new_height > temp[temp>0.0].mean():
                self.new_height = temp[temp>0.0].mean()

            # Average the depth values to reduce error
            temp = self.depth_full[self.cx-5:self.cx+5,self.cy-5:self.cy+5]
            self.base_height = temp[temp>0.0].mean()

            # Calculate height of object
            if self.base_height > self.new_height:
                height = self.base_height - self.new_height

            # Display
            if self.cy >= self.cyF-20 and self.cy <= self.cyF+20 and self.cx >= 100 and self.cx <= 300:    
                print("height_box : ", height)
                print("volume : ", height*self.area)

        if self.calibFlag2 == True:
            self.depth_image.unregister()
            self.calibFlag2 = False



    def rgbCallback(self, rgb):
        # Callback function to process the rgb camera data

        ## Image to cv_image
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
        except CvBridgeError:
            rospy.loginfo("rgb failed")

        self.area = 0
        ## Grayscale & detection
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7,7), 0)

        edged = cv2.Canny(gray, 50, 100)
        edged = cv2.dilate(edged, None, iterations = 1)
        edged = cv2.erode(edged, None, iterations = 1)

        orig = rgb_image.copy()

        cnts = cv2.findContours(edged, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Contours and their moments
        (cnts, _) = contours.sort_contours(cnts)
        for cnt in cnts:
            M = cv2.moments(cnt)
            cx = 0
            cy = 0
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

            # Reference object
            if cv2.contourArea(cnt) > 1000 and cv2.contourArea(cnt) < 3000:
                #if cy >= 100 and cy <= 400 and cx >= 100 and cx <= 400 and self.pixelPerMetricA is None: # moving reference
                if cy >= 100 and cy <= 400 and cx >= 100 and cx <= 400 and self.ppmA is None: # static reference
                    #self.pixelPerMetricA, self.pixelPerMetricB, _ = image_test().boxMark(cnt, orig, self.pixelPerMetricA, self.pixelPerMetricB) # moving reference
                    image_test().boxMark(cnt, orig) # static reference
                    self.cyF = cy
                    self.cxF = cx
                continue

            # Object whose volume is to be calculated
            # We ignore objects under 5000 pixel**2 to reduce unnecessary detection
            elif cv2.contourArea(cnt) > 5000:   
                if cy >= self.cyF-20 and cy <= self.cyF+20 and cx >= 100 and cx <= 300:
                    self.cx = cx
                    self.cy = cy
                    self.flag = True
                    self.area = image_test().boxMark(cnt, orig) # static reference
                    # minimum area
                    if self.area < self.area:
                        self.area = self.area
                    print("area : ", self.area)
                    #print("volume : ", self.area)

                # Signal that object is out of range and reset values
                elif cy >= self.cyF+75:
                    self.new_height = 100
                    self.area = 100000
                    self.flag = False
                    print("reset")

        cv2.rectangle(orig, (100, self.cyF-20), (300, self.cyF+20), ( 0, 0, 255), 1)
        cv2.rectangle(orig, (100, 100), (300, self.cyF+75), ( 0, 255, 255), 1)
        
        cv2.imshow("rgb_view", orig)
        cv2.waitKey(1)

        if self.calibFlag1 == True:
            self.image_rgb.unregister()
            self.calibFlag1 = False

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()
            

def main(argv):

    rospy.init_node('volumetric_kinect', anonymous = True)
    ic = image_test()

    isCalib = input("Do you want to use the default calibration? (y/n) :")
    if isCalib.lower() == 'n':
        ic.calibration()

    print("Proceeding . . . " )

    ic.readCalibration()

    try:
        ic.rgbListener()
        #rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main(sys.argv)