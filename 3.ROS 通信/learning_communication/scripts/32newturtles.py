#!/usr/bin/env python

import rospy

import turtlesim.msg
 
from turtlesim.srv import Spawn,SpawnRequest
#from geometry_msgs.msg import Twist


def newturtle_client():
    # init node
    rospy.init_node('newturtle_client')

    # spawnClient = rospy.Service

    
    rospy.wait_for_service('/spawn')
    try:
        # build a client, turtlrCallback
        spawnClient = rospy.ServiceProxy('/spawn',Spawn)
        
	'''       
        # build turtlesim.srv.Spawn service
        srv = SpawnRequest() # import
        srv.x = 5.0
        srv.y = 6.0
        srv.theta = 2.0
        #srv.name = 'turtle11'
        
        # Publish service request, wait for response result
        if(spawnClient.call(srv)):
            #print "A new turtle named '%s' was Spawned", srv.name()          
            print "A new turtle was Spawned"
            #print "sss"
        else:
            print "Failed to call service Spawn"
        '''
        spawnClient(6.0, 6.0, 2.0, 'alpha1')
        #return Spawn.name
    except rospy.ServiceException,e:
        print "Service call faild: %s" %e


if __name__ == "__main__":
    newturtle_client()
