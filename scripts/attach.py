#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int32 
from sensor_msgs.msg import JointState
import math
from geometry_msgs.msg import Point
#
objectivePoint= Point()
#
jointstates = JointState()
#
myModelStates = ModelStates() 
#
newState = 0
stateAtach = 0
#
newModelState = ModelState()
newModelState.model_name=''
newModelState.pose.position.x=0.65
newModelState.pose.position.y=-0.475
newModelState.pose.position.z=0.988761+.9
newModelState.twist.linear.x=0
newModelState.twist.linear.y=0
newModelState.twist.linear.z=0
newModelState.twist.angular.x=0
newModelState.twist.angular.y=0
newModelState.twist.angular.z=0
newModelState.reference_frame='world'
#
teta1 = 0
teta2 = 0
teta3 = 0
teta4 = 0
#
x = 0
y = 0
z = 0
# move arm
def gripper_open_callback(message):
    global newState
    global stateAtach
    newState = 1
    stateAtach = message.data
def gazebo_states_callback(message):
    global myModelStates
    global objectivePoint
    global newModelState
    global stateAtach
    myModelStates.name = message.name
    myModelStates.pose = message.pose
    myModelStates.twist = message.twist
    oldDistance = 100
    newDistance = 200
    counter=0
    if stateAtach:
        for c in myModelStates.name :
            if (c.find("ball")>=0 or c.find("cube")>=0):
                newDistance = math.sqrt(math.pow((objectivePoint.x-myModelStates.pose[counter].position.x),2)+math.pow((objectivePoint.y-myModelStates.pose[counter].position.y),2)) 
                if newDistance < oldDistance:
                    oldDistance = newDistance
                    newModelState.model_name=c
            counter=counter+1
            
def robot_link_states_callback(message):
    global jointstates
    global teta1 
    global teta2 
    global teta3 
    global teta4 
    global x
    global y
    global z
    global newModelState
    jointstates.name = message.name
    jointstates.position = message.position
    jointstates.velocity = message.velocity
    jointstates.effort = message.effort
    teta1 = jointstates.position[0]
    teta2 = jointstates.position[1]
    teta3 = jointstates.position[2]
    teta4 = jointstates.position[3]
    #x = -(math.sin(teta1)*(6*math.cos(teta2 + teta3) - 5*math.sin(teta2)))/10
    x = (.6*math.cos(teta2) +.65*math.cos(teta3+teta2))*math.cos(teta1)
    y = (.6*math.cos(teta2) +.65*math.cos(teta3+teta2))*math.sin(teta1)
    z = (.6*math.sin(teta2) +.65*math.sin(teta3+teta2))
    z =z +.6+0.988761-0.015
    try:
        newModelState.pose.position.x=x
        newModelState.pose.position.y=y
        newModelState.pose.position.z= z
        teta4=teta4+3.14159260597
        m00=math.cos(teta1)*math.cos(teta4) - math.sin(teta4)*(math.cos(teta2)*math.sin(teta1)*math.sin(teta3) + math.cos(teta3)*math.sin(teta1)*math.sin(teta2))
        m11=math.cos(teta2 + teta3)*math.cos(teta1)
        m22=math.cos(teta2 + teta3)*math.cos(teta4)
        qw= math.sqrt(1 + m00 + m11 + m22) /2
        m21=math.sin(teta2 + teta3)
        m12=math.sin(teta1)*math.sin(teta4) - math.cos(teta4)*(math.cos(teta1)*math.cos(teta2)*math.sin(teta3) + math.cos(teta1)*math.cos(teta3)*math.sin(teta2))
        qx=(m21 - m12)/( 4 *qw)
        m02=math.cos(teta1)*math.sin(teta4) + math.cos(teta4)*(math.cos(teta2)*math.sin(teta1)*math.sin(teta3) + math.cos(teta3)*math.sin(teta1)*math.sin(teta2)) 
        m20=-math.cos(teta2 + teta3)*math.sin(teta4)
        qy = (m02 - m20)/( 4 *qw)
        m10=math.cos(teta4)*math.sin(teta1) + math.sin(teta4)*(math.cos(teta1)*math.cos(teta2)*math.sin(teta3) + math.cos(teta1)*math.cos(teta3)*math.sin(teta2))
        m01=-math.cos(teta2 + teta3)*math.sin(teta1)
        qz = (m10 - m01)/( 4 *qw)
        newModelState.pose.orientation.x = qx
        newModelState.pose.orientation.y = qy
        newModelState.pose.orientation.z = qz
        newModelState.pose.orientation.w = qw
    except:
        newModelState.pose.orientation.x = 0
        newModelState.pose.orientation.y = 0
        newModelState.pose.orientation.z = 0
        newModelState.pose.orientation.w = 0

def Point_callback(message):
    global objectivePoint
    objectivePoint.x=message.x
    objectivePoint.y=message.y
    objectivePoint.z=message.z


#main
def attach():
    global newState
    global stateAtach

    global newModelState
    global myModelStates
    #set up node
    rospy.init_node('attachLogic', anonymous=True)
    #set up publisher
    pubOA = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
    rate = rospy.Rate(100)
    # Set up subscriber
    rospy.Subscriber("/gripper/open", Int32, gripper_open_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_states_callback)
    rospy.Subscriber("/robot/joint_states", JointState, robot_link_states_callback)
    rospy.Subscriber("/armMovent/Point", Point, Point_callback)



    #Publish data to motors
    while not rospy.is_shutdown():
        if stateAtach == 0: 
            pubOA.publish(newModelState)
            print (newModelState.model_name)
        rate.sleep()


if __name__ == '__main__':

    attach()
