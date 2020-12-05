#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import Int32 
import math
#global variable
#ros puvblisher
objectPoint = Point()
objectType = 0
#image processing
imgResult = []
myColors = [[0, 0,133,179,  8,188], #Light Gray - Floor
            [0,53, 96,  6,255,255], #Red        - Spheres
            [9,54,145, 20,255,255]] #Orange     - Cubes

colorNum=3;                         #2 Colors and the floor 
BNImages=[]
BNImages_Prs=[]
Points=[]                           #[ColorNum or Figure][ElementNum][x,y]
Espheres = []
Cubes = []

class image_processing:
  def __init__(self):
    #Constructor
    #Define publisher
    self.image_pub = rospy.Publisher("/cv2/image",Image,queue_size=1)
    self.object_point_pub = rospy.Publisher("/object/point",Point,queue_size=1)
    self.object_type_pub = rospy.Publisher("/object/type",Int32,queue_size=1)
    #bdridge ros open cb
    self.bridge = CvBridge()
    #subscriber
    self.image_sub = rospy.Subscriber("/camera1/image_raw",Image,self.callback)
  def callback(self,data):
    # call global variables
    global imgResult
    global colorNum                         #2 Colors and the floor 
    global BNImages
    global BNImages_Prs
    global Points                         #[ColorNum or Figure][ElementNum][x,y]
    global Espheres 
    global Cubes 
    global myColors
    global objectPoint
    global objectType
    # convert image
    try:
        src = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print("")

    #clear variables
    try:
        del Cubes[:]
        del Espheres[:]
        del Points[:]
        del BNImages [:]
        del BNImages_Prs [:]
    except:
        print ("Empty")
    try :
        #Proces the image
        imgResult=src.copy()
        ###
        findColor(src) 
        ####-----------------------OpenClose Noise reduction--------
        noiseReduction(BNImages)
        ###----------------------Countours and points-------------
        getCenter(BNImages_Prs)
        ###----------------------Tomar Medidas Segun pixeles-------
        getRealDimensions(BNImages_Prs)
    except:
        print ("ROS")
    #get the objective point
    cordenatePoint(Cubes,Espheres)

    print ("Espheres:")
    print (Espheres)
    print ("\n")
    print ("Cubes:")
    print (Cubes)
    print ("\n")
    #publish points
    self.object_point_pub.publish(objectPoint)
    self.object_type_pub.publish(objectType)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(imgResult, "bgr8"))
    except CvBridgeError as e:
      print("")

def cordenatePoint(Cubes,Espheres):
    global objectPoint
    global objectType
    objectType = 0
    objectPoint.x = 0
    objectPoint.y = 0
    objectPoint.z = 0
    oldSize = 0
    newSize = 0
    for c in Cubes :
        newSize =math.sqrt(math.pow(c[0],2)+math.pow(c[1],2)) 
        if newSize > oldSize:
            oldSize=newSize
            objectPoint.x = c[0]
            objectPoint.y = c[1]
            objectPoint.z = 0.025
            objectType = 2
    for c in Espheres :
        newSize =math.sqrt(math.pow(c[0],2)+math.pow(c[1],2)) 
        if newSize > oldSize:
            oldSize=newSize
            objectPoint.x = c[0]
            objectPoint.y = c[1]
            objectPoint.z = 0.025
            objectType = 1
def empty(a): #To be used with trckpath(img)
    pass

def trckpath(img): #Only to be used for detecting wich colors are inside test image
    m=1
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars", 640, 240)
    cv2.createTrackbar("Hue Min", "TrackBars",   0, 179, empty)
    cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
    cv2.createTrackbar("Sat Min", "TrackBars",   0, 255, empty)
    cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
    cv2.createTrackbar("Val Min", "TrackBars",   0, 255, empty)
    cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)
    while m != 0:
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
        
        lowerLim = np.array([h_min,s_min,v_min])
        upperLim = np.array([h_max,s_max,v_max])
        mask = cv2.inRange(imgHSV,lowerLim,upperLim)
        
        cv2.imshow("Source",src)
        cv2.imshow("HSV",imgHSV)
        cv2.imshow("Mask",mask)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 'q':
            m=0

def findColor(img):
    global colorNum                         #2 Colors and the floor 
    global BNImages
    global BNImages_Prs
    global Points                         #[ColorNum or Figure][ElementNum][x,y]
    global Espheres 
    global Cubes 
    global myColors
    heigh,width,depht=img.shape
    imgHsv = cv2.cvtColor(img[0:(int)(heigh*0.95),0:width], cv2.COLOR_BGR2HSV)
    #cv2.imshow("HSV",imgHsv)
    for color in myColors:
        lower = np.array(color[0:3])
        upper = np.array(color[3:6])
        mask = cv2.inRange(imgHsv,lower,upper)
        BNImages.append(mask)
        #cv2.imshow(str(color),mask)

def noiseReduction(Images):
    global colorNum                         #2 Colors and the floor 
    global BNImages
    global BNImages_Prs
    global Points                         #[ColorNum or Figure][ElementNum][x,y]
    global Espheres 
    global Cubes 
    i=0
    kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT,(4,4))
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    for img in Images:
        i=i+1
        Prc = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel2)
        Prc = cv2.morphologyEx(Prc, cv2.MORPH_CLOSE, kernel1)

        BNImages_Prs.append(Prc)
        #cv2.imshow(str(i),Prc)

def getCenter(Images):
    global colorNum                         #2 Colors and the floor 
    global BNImages
    global BNImages_Prs
    global Points                         #[ColorNum or Figure][ElementNum][x,y]
    global Espheres 
    global Cubes 
    global imgResult
    for img in Images[1:]:
        # find contours in the binary image
        im2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        ElementPoints = []
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
              
            # calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            cv2.circle(imgResult, (cX, cY), 15, (255, 0, 0), 1)
            ElementPoints.append([cX,cY])
        Points.append(ElementPoints)

def getRealDimensions(Images):
    global colorNum                         #2 Colors and the floor 
    global BNImages
    global BNImages_Prs
    global Points                         #[ColorNum or Figure][ElementNum][x,y]
    global Espheres 
    global Cubes 
    global imgResult
    table = cv2.bitwise_not(Images[0])
    im2, contours, hierarchy = cv2.findContours(table,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    x,y,w,h = cv2.boundingRect(contours[0])
    cv2.circle(imgResult, (x, y), 5, (0, 255, 0), -1)
    cv2.circle(imgResult, (x+w, y), 5, (0, 255, 0), -1)
    
    for Elements in Points[0]:  #Esferas
        a=float(Elements[0]-x-float(w)/2)*((2*0.8738976372)/float(w))+0.035913601528767124

        b=1.3-float(Elements[1]-y)*(2*(0.95/float(w)))+0.046076342465733
        Espheres.append([b,-a])
        # CoordText="(" + str(a) + ", " + str(b) + ")"
        # cv2.putText(imgResult, CoordText, (Elements[0],Elements[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1) 
    for Elements in Points[1]:  #Cubos
        a=float(Elements[0]-x-float(w)/2)*((2*0.9246325457)/float(w))+0.006333099628082191


        b=1.3-float(Elements[1]-y)*(2*(0.95/float(w)))+0.0494863013698629
        Cubes.append([b,-a])
        # CoordText="(" + str(a) + ", " + str(b) + ")"
        # cv2.putText(imgResult, CoordText, (Elements[0],Elements[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1) 
                


#-0.537556
#0.519813
def main():
    #create node
    rospy.init_node('vision', anonymous=True)
    #create clas that procces the imgae
    ic = image_processing()
    #make a bucle so the program dosent end
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()