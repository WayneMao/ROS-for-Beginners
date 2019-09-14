#!/usr/bin/env python

import rospy
import turtlesim.msg
 
from turtlesim.srv import Spawn,SpawnRequest
from geometry_msgs.msg import Twist

import sys
import random
import string
import time

def checkinput(cmd):
    print "checkinput"
    try:
        name,velocity,angular = cmd.split()
        #print "The turtle named %s "
        linearx = float(velocity)
        angularz = float(angular)
        print "The turtle named %s, linear velocity: %f, angular:%f"%(name, linearx, angularz)
        pub = rospy.Publisher('/%s/cmd_vel'%name,Twist, queue_size = 10)
        #delay
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #init Twist
            twist = Twist()
            twist.linear.x = linearx
            twist.angular.z = angularz
            # publish message
            pub.publish(twist)
            rate.sleep()

    except ValueError:
        try:
            new,name = cmd.split()
            print "Spawn a new turtle named %s" %name
            newturtle_client(name) ## random spawn a new trrtle
        except ValueError:
            cmd = cmd
            print "%s"%cmd
            #newturtle_client()
            control(cmd)

def control(cmd):
    if cmd == 'stop' or cmd == 'STOP':
        print "The turtle successfully stopped"
    elif cmd == 'start' or cmd == 'START':
        print "start"

def newturtle_client(name):
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
        rospy.loginfo('Successfully produced a new turtle')
    except rospy.ServiceException,e:
        print "Service call faild: %s" %e


if __name__ == "__main__":    
    # init node 
    rospy.init_node('newturtle_client') 
    
    # rate = rospy.Rate(10)
    print "Input command: 3 parameters means name, velocity, angular.\
           One parameter: stop,start;\
           two paramaters: spawn name. The name is new turtles' name."
    try:
        while not rospy.is_shutdown():
            print "Please input command"
            cmd = raw_input()
            if cmd== None:
                print "Please input command"
            checkinput(cmd)
            #rate.sleep()
            #time.sleep(50)
    except KeyboardInterrupt:
        pass

