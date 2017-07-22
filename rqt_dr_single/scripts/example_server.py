#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from rqt_dr_single.cfg import ExampleConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {enum_param}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("rqt_dr_single", anonymous = True)

    srv = Server(ExampleConfig, callback)
    rospy.spin()
