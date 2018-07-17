#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure_example.cfg import ExampleConfig

count = 0
hangup = False

def callback(config, level):
    global count
    global hangup
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {big_double}, {str_param}, {bool_param}, {enum_param}""".format(**config))

    if hangup:
        count += 1
        if count > 2:
            rospy.loginfo("going to sleep for a while")
            rospy.sleep(10.0)
    return config

if __name__ == "__main__":
    rospy.init_node("rqt_dr_single", anonymous = True)

    hangup = rospy.get_param("~hangup", False)
    srv = Server(ExampleConfig, callback)
    rospy.spin()
