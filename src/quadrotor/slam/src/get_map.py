#!/usr/bin/env python

import sys
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap 


if __name__=="__main__":
    if len(sys.argv)==2:
        name = sys.argv[1]

        rospy.wait_for_service("/{}/get_map".format(name))                      # Wait for service that return the map
        service_call = rospy.ServiceProxy("/{}/get_map".format(name), GetMap)       
        try:
            my_map = service_call()                         #Require the map
            rospy.loginfo("Result = {}".format(my_map))
        except rospy.ROSInterruptException:
            pass
    else:
        print("Wrong statement.")
