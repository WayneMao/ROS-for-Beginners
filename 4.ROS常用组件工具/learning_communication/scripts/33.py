#!/usr/bin/env python

import rospy
import turtlesim.msg
 
from turtlesim.srv import Spawn,SpawnRequest
from geometry_msgs.msg import Twist

import sys
import random

def newturtle_client():
    # init node
    rospy.init_node('newturtle_client')
    # wait service
    rospy.wait_for_service('/spawn')
    
    # random reduce float
    x = random.uniform(0,9)
    y = random.uniform(0,9)
    z = random.uniform(0,9)
    rospy.loginfo('This turtle\'s name is %s.', name)
    try:
        # build a client, turtlrCallback
        spawnClient = rospy.ServiceProxy('/spawn',Spawn)
        spawnClient(x, y, z, name)
    except rospy.ServiceException,e:
        print "Service call faild: %s" %e

def control_publisher():
    # init node
    #rospy.init_node('turtle_cmd', anonymous = True)
    
    # get input 
    name1,linearx, angularz = raw_input("Please input turtle\'s name, linear_velocity and angular: ").split( )
    
    #print "Please input turtle\'s name, linear_velocity and angular:"
    #name1,linearx,angularz = rospy.get_param(name1)
    
    linearx = float(linearx)
    angularz = float(angularz)
    print 'linear velocity: %f; angular: %f' %(linearx, angularz)

    # build a publisher
    pub = rospy.Publisher('/%s/cmd_vel' %name1, Twist, queue_size = 10)

    # delay
    rate = rospy.Rate(10)

    # loop
    while not rospy.is_shutdown():
        # init Twist
        twist = Twist()
        twist.linear.x = linearx 
        twist.angular.z = angularz 

        # publish message
        pub.publish(twist)

        rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'Please input turtle name.'
    else:
        name = sys.argv[1]
        bbb = rospy.get_param('bbb')
        print bbb
        newturtle_client()
        control_publisher()









