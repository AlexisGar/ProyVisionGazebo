#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32 
from geometry_msgs.msg import Point
#Point
home = Point()
home.x = -0.640
home.y = 0
home.z = 0.6
dropCube = Point()
dropCube.x = -0.65
dropCube.y = -0.45
dropCube.z = 0.8
dropPelota = Point()
dropPelota.x = -0.65
dropPelota.y = 0.45
dropPelota.z = 0.8
#Object data
objectPoint = Point()
objectPointT = Point()

newObjectPoint   =   0
objectType =  0
newObjecType  =   0

#Object types
pelota  =   1
cube    =   2
#Status
armMoventSatus  =   0
gripperStatus   =   0
armPathStatus   =   0
# path pointer
pathPointer = 0

def object_Point_callback(message):
    global objectPoint
    global newObjectPoint
    #update only after path has ended
    if newObjectPoint == 0 :
        objectPoint.x = message.x
        objectPoint.y = message.y
        objectPoint.z = message.z
        newObjectPoint =1
def object_Type_callback(message):
    global objectType
    global newObjecType
    #update only after path has ended
    if newObjecType == 0 :
        objectType = message.data
        newObjecType = 1
def armMovent_Status_callback(message):
    global armMoventSatus
    armMoventSatus = message.data

def gripper_Status_callback(message):
    global gripperStatus
    gripperStatus = message.data
#main
def givePath():
    #Points
    global home
    global dropCube
    global dropPelota
    #Object data
    global objectPoint
    global objectPointT
    global newObjectPoint
    global objectType
    global newObjecType

    #Object types
    global pelota
    global cube
    #Status
    global armMoventSatus
    global gripperStatus
    global armPathStatus
    # path pointer
    global pathPointer



    #set up node
    rospy.init_node('armPathLogic', anonymous=True)
    #set up publisher
    pubAMP = rospy.Publisher('/armMovent/Point', Point, queue_size=10)

    pubGO  = rospy.Publisher('/gripper/open', Int32, queue_size=10)
    pubAPS = rospy.Publisher('/armPath/status', Int32, queue_size=10)
    #start home
    #pubAMP.publish(home)
    rate = rospy.Rate(1)
    # Set up subscriber
    rospy.Subscriber("/object/point", Point, object_Point_callback)
    rospy.Subscriber("/object/type", Int32, object_Type_callback)
    rospy.Subscriber("/armMovent/status", Int32, armMovent_Status_callback)
    rospy.Subscriber("/gripper/status", Int32, gripper_Status_callback)
    
    start = 0
    #Publish data to motors
    while not rospy.is_shutdown():
        if start < 2:
            #go home
            start = start+1
            pubAMP.publish(home)
        #check that a new point arrive 
        if (newObjectPoint == 1) and (newObjecType ==1) :
            #check if is a valid object
            if (objectType == cube ) or (objectType == pelota ) :
                #move to home and open gipper
                if pathPointer == 0 :
                    #go home
                    pubAMP.publish(home)
                    #open griper
                    pubGO.publish(1)
                    #increasePathPointer
                    pathPointer =   pathPointer +   1
                    armPathStatus = 1
                #move abode the object
                elif pathPointer == 1 and armMoventSatus == 0:
                    objectPointT.x = objectPoint.x
                    objectPointT.y = objectPoint.y
                    objectPointT.z = objectPoint.z + 0.080
                    pubAMP.publish(objectPointT)
                    pathPointer =   pathPointer +   1
                #move to object
                elif pathPointer == 2 and armMoventSatus == 0:
                    objectPointT.x = objectPoint.x
                    objectPointT.y = objectPoint.y
                    objectPointT.z = objectPoint.z + 0.005
                    pubAMP.publish(objectPointT)
                    pathPointer =   pathPointer +   1
                #close gripper
                elif pathPointer == 3 and armMoventSatus == 0:
                    pubGO.publish(0)
                    pathPointer =   pathPointer +   1
                #move abode the object
                elif pathPointer == 4 and gripperStatus == 0:
                    objectPointT.x = objectPoint.x
                    objectPointT.y = objectPoint.y
                    objectPointT.z = objectPoint.z + 0.080
                    pubAMP.publish(objectPointT)
                    pathPointer =   pathPointer +   1
                #move to drop
                elif pathPointer == 5 and armMoventSatus == 0:
                    if (objectType == cube ) :
                        pubAMP.publish(dropCube)
                        pathPointer =   pathPointer +   1
                    elif(objectType == pelota ) :
                        pubAMP.publish(dropPelota)
                        pathPointer =   pathPointer +   1
                #open gripper
                elif pathPointer == 6 and armMoventSatus == 0:
                    pubGO.publish(1)
                    pathPointer =   pathPointer +   1
                #move to home
                elif pathPointer == 7 and gripperStatus == 0:
                    #go home
                    pubAMP.publish(home)
                    #reset flags
                    pathPointer     = 0
                    armPathStatus   = 0
                    newObjectPoint  = 0 
                    newObjecType    = 0
        pubAPS.publish(armPathStatus)
        rate.sleep()




if __name__ == '__main__':
    givePath()
