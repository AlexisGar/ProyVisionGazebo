#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 
from control_msgs.msg import JointControllerState
from std_msgs.msg import Int32 

# to khow if a new point arrive
newSatete = 0
#Objective status
inMovent = 0
#Objective angles
    #Link0-Link1
lenghtLObjective = 0
    #Link1-Link2
lenghtRObjective = 0


griperFingerObjective = 0
#Current angles
lenghtLCurrent = 0
lenghtRCurrent = 0
error = 0.001


# move arm
def gripper_open_callback(message):
    global newSatete
    global inMovent
    #movent fingers
    global lenghtLObjective
    global lenghtRObjective
    # arm and reference frame parameters
    try :
        if message.data == 1 :
            opening = 0.05
            lenghtLObjective = opening
            lenghtRObjective = opening
            newSatete = 1
            inMovent = 1
        elif message.data == 0 :
            close = 0.05
            lenghtLObjective = close
            lenghtRObjective = close
            newSatete = 1
            inMovent = 1
    except :
        newSatete = 0
        inMovent = 0
# satus movent
def gripper_l_state_callback(message):
    global lenghtLCurrent
    lenghtLCurrent = message.process_value
def gripper_r_state_callback(message):
    global lenghtRCurrent
    lenghtRCurrent = message.process_value

#main
def openGriper():
    #movent fingers
    global lenghtLObjective
    global lenghtRObjective 
    # current Lenght
    global lenghtLCurrent
    global lenghtRCurrent
    #robot is in movent
    global inMovent
    global newSatete
    #set up node
    rospy.init_node('gripperLogic', anonymous=True)
    #set up publisher
    pubGL = rospy.Publisher('/robot/lgrip/command', Float64, queue_size=10)
    pubGR = rospy.Publisher('/robot/rgrip/command', Float64, queue_size=10)
    pubGS = rospy.Publisher('/gripper/status', Int32, queue_size=10)

    rate = rospy.Rate(1)
    # Set up subscriber
    rospy.Subscriber("/gripper/open", Int32, gripper_open_callback)
    rospy.Subscriber("/robot/lgrip/state", JointControllerState, gripper_l_state_callback)
    rospy.Subscriber("/robot/rgrip/state", JointControllerState, gripper_r_state_callback)

    #Publish data to motors
    while not rospy.is_shutdown():
        if newSatete : 
            pubGL.publish(lenghtLObjective)
            pubGR.publish(lenghtRObjective)
            newSatete = 0
            inMovent = 1
            rate.sleep()
        if abs(lenghtLObjective - lenghtLCurrent) <= error :
            if abs(lenghtRObjective - lenghtRCurrent) <= error:
                inMovent = 0
        pubGS.publish(inMovent)
        rate.sleep()


if __name__ == '__main__':

    openGriper()
