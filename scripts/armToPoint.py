#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 
from std_msgs.msg import Int32 
from geometry_msgs.msg import Point
from control_msgs.msg import JointControllerState
import math
import time
# to khow if a new point arrive
newPoint = 0
#Objective status
inMovent = 0
#Objective angles
    #Link0-Link1
teta1Objective = 0
    #Link1-Link2
teta2Objective = 0
    #Link2-Link3
teta3Objective = 0
    #Link3-Link4
teta3Objective = 0

griperFingerObjective = 0
#Current angles
teta1Current = 0
teta2Current = 0
teta3Current = 0
griperFingerCurrent = 0
error = 0.005

#Joint size
l1 = .6
l2 = .6505
h  = .6 
# move arm
def to_Point_callback(message):
    global newPoint
    global inMovent
    #movent angles
    global teta1Objective
    global teta2Objective 
    global teta3Objective 
    global teta4Objective 

    # arm and reference frame parameters
    global l1
    global l2
    global h
    # movent tolerance error 
    global error
    try :
        #angle link 1
        teta1Objective = math.atan2(message.y,message.x)

        teta4Objective = teta1Objective
        #angle link 2
        s = math.pow(message.x,2) + math.pow(message.y,2) 
        s = math.pow(s,.5) 
        h2 = message.z -h
  
        alfa = math.acos( (math.pow(l1,2) - math.pow(l2,2) + math.pow(s,2) + math.pow(h2,2)) / (2 * l1 * math.pow((math.pow(s,2)+math.pow(h2,2)),.5)) )
        #print alfa
        teta2Objective =  alfa + math.atan2(h2,s) 
        #print teta2Objective
        beta = math.acos( (math.pow(l1,2) + math.pow(l2,2) - math.pow(s,2) - math.pow(h2,2)) / (2 * l1 * l2) )
        #angle link 3
        teta3Objective = (beta - (math.pi))


        if teta2Objective <-0.7:
            teta2Objective = -0.76
        elif teta2Objective > 1.57079632:
            teta2Objective = 1.57079632
        if teta3Objective <-1.57079632:
            teta3Objective = -1.57079632
        elif teta3Objective > 1.57079632:
            teta3Objective = 1.57079632
        #print teta3Objective
        newPoint = 1
        inMovent = 1
    except :
        newPoint = 0
        inMovent = 0
# satus movent
def joint1_state_callback(message):
    global teta1Current
    teta1Current = message.process_value
def joint2_state_callback(message):
    global teta2Current
    teta2Current = message.process_value
def joint3_state_callback(message):
    global teta3Current
    teta3Current = message.process_value
#main
def toPoint():
    # a new point was got flag
    global newPoint
    # revolution movent angle
    global teta1Objective
    global teta2Objective
    global teta3Objective
    global teta4Objective

    # current angle
    global teta1Current
    global teta2Current
    global teta3Current
    #robot is in movent
    global inMovent

    #set up node
    rospy.init_node('armLogic', anonymous=True)
    #set up publisher
    pubJ1 = rospy.Publisher('/robot/joint1/command', Float64, queue_size=10)
    pubJ2 = rospy.Publisher('/robot/joint2/command', Float64, queue_size=10)
    pubJ3 = rospy.Publisher('/robot/joint3/command', Float64, queue_size=10)
    pubJ4 = rospy.Publisher('/robot/joint4/command', Float64, queue_size=10)
    pubAS = rospy.Publisher('/armMovent/status', Int32, queue_size=10)

    rate = rospy.Rate(1)
    # Set up subscriber
    rospy.Subscriber("/armMovent/Point", Point, to_Point_callback)
    rospy.Subscriber("/robot/joint1/state", JointControllerState, joint1_state_callback)
    rospy.Subscriber("/robot/joint2/state", JointControllerState, joint2_state_callback)
    rospy.Subscriber("/robot/joint3/state", JointControllerState, joint3_state_callback)

    #Publish data to motors
    while not rospy.is_shutdown():
        if newPoint : 
            pubJ1.publish(teta1Objective)
            pubJ2.publish(teta2Objective)
            pubJ3.publish(teta3Objective)
            pubJ4.publish(teta4Objective)
            newPoint = 0
            inMovent = 1
        """
        if abs(teta1Objective - teta1Current) <= error :
            if abs(teta2Objective - teta2Current) <= error:
                if abs(teta3Objective - teta3Current) <= error:
                    inMovent = 0
        """
        if inMovent:
            inMovent=0
            time.sleep(3)

        rate.sleep()
                    
        pubAS.publish(inMovent)


if __name__ == '__main__':
    toPoint()
