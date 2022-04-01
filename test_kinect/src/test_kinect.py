#!/usr/bin/env python

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

from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import imutils
import cProfile
import re


class image_test:

    base_height = 0.0
    new_height = 100
    area = 10000
    cyF = 0
    cxF = 0
    ppmA = None
    ppmB = None
    flag2 = False
    
    def __init__(self):

        #global flag1
        global flag2
        self.t_old = time()
        self.depth_full = None

        ## variable for static reference
        #self.pixelPerMetricA = None
        #self.pixelPerMetricB = None

        self.bridge = CvBridge()

        self.image_depth = message_filters.Subscriber("/camera/depth/image", Image)
        self.image_rgb = message_filters.Subscriber("/camera/rgb/image_color", Image)

    def midpoint(self, ptA, ptB):
	    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

    #def boxMark(self, cnt, orig, pixelPerMetricA, pixelPerMetricB): # moving reference
    def boxMark(self, cnt, orig): ## static reference
        #time_now = time()
        box = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)
                
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
        dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
        dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
        
        #if pixelPerMetricA is None and pixelPerMetricB is None: # moving reference
        #    pixelPerMetricA = dA/0.06
        #    pixelPerMetricB = dB/0.06
        #dimA = dA/pixelPerMetricA
        #dimB = dB/pixelPerMetricB

        if image_test.ppmA is None and image_test.ppmB is None: # static reference
            image_test.ppmA = dA/0.06   # 6cm x 6cm reference
            image_test.ppmB = dB/0.06
        dimA = dA/image_test.ppmA
        dimB = dB/image_test.ppmB
        
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

    def test_callback(self, depth, rgb):
        
        ## Image to cv_image
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
        except CvBridgeError:
            rospy.loginfo("rgb failed")

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
        except CvBridgeError:
            rospy.loginfo("depth failed")

        ## cv_image to array
        if self.depth_full is None:
            self.depth_full = np.array(depth_image, dtype = np.dtype('float32'))
            print "Init done"

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

        (cnts, _) = contours.sort_contours(cnts)
        for cnt in cnts:
            M = cv2.moments(cnt)
            cx = 0
            cy = 0
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            if cv2.contourArea(cnt) > 1000 and cv2.contourArea(cnt) < 3000:
                #if cy >= 100 and cy <= 400 and cx >= 100 and cx <= 400 and self.pixelPerMetricA is None: # moving reference
                if cy >= 100 and cy <= 400 and cx >= 100 and cx <= 400 and image_test.ppmA is None: # static reference
                    #self.pixelPerMetricA, self.pixelPerMetricB, _ = image_test().boxMark(cnt, orig, self.pixelPerMetricA, self.pixelPerMetricB) # moving reference
                    image_test().boxMark(cnt, orig) # static reference
                    image_test.cyF = cy
                    image_test.cxF = cx
                continue
            elif cv2.contourArea(cnt) > 5000:   
                if cy >= image_test.cyF-20 and cy <= image_test.cyF+20 and cx >= 100 and cx <= 300:
                    #image_test.flag2 = True
                    temp = np.zeros((10,10), dtype=np.dtype('float32'))
                    self.depth_new = np.array(depth_image, dtype = np.dtype('float32'))
                    temp = self.depth_new[cx-5:cx+5,cy-5:cy+5]
                   
                    if image_test.new_height > temp[temp>0.0].mean():
                        image_test.new_height = temp[temp>0.0].mean()

                    temp = self.depth_full[cx-5:cx+5,cy-5:cy+5]
                    image_test.base_height = temp[temp>0.0].mean()

                    #print ppmA, ppmB
                    #_, _, self.area = image_test().boxMark(cnt, orig, self.pixelPerMetricA, self.pixelPerMetricB) # moving reference
                    self.area = image_test().boxMark(cnt, orig) # static reference

                    if self.area < image_test.area:
                        image_test.area = self.area

                    if image_test.base_height > image_test.new_height:
                        height = image_test.base_height - image_test.new_height
                        print "height_box : ", height
                        print "area : ", image_test.area
                        print "volume : ", image_test.area*height

                elif cy >= image_test.cyF+75:
                    image_test.new_height = 100
                    image_test.area = 100000
                    print "reset"

        cv2.rectangle(orig, (100, image_test.cyF-20), (300, image_test.cyF+20), ( 0, 0, 255), 1)
        cv2.rectangle(orig, (100, 100), (300, image_test.cyF+75), ( 0, 255, 255), 1)
        
        cv2.imshow("rgb_view", orig)
        cv2.waitKey(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()
            

def main(argv):

    rospy.init_node('volumetric_kinect', anonymous = True)
    ic = image_test()

    try:
        ts = message_filters.ApproximateTimeSynchronizer([ic.image_depth,
        ic.image_rgb], 10, 1)
        ts.registerCallback(ic.test_callback)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main(sys.argv)