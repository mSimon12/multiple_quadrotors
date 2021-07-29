#!/usr/bin/env python2.7

import rospy

if __name__=="__main__":
    rospy.init_node("changer")
    namespace = rospy.get_namespace()
    sensors = rospy.get_param("{}move_group/sensors".format(namespace))
    
    sensors[0]['point_cloud_topic'] = "{}camera/depth/points".format(namespace)

    rospy.set_param("{}move_group/sensors".format(namespace),sensors)
