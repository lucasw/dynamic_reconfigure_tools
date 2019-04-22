#!/usr/bin/env python

import rospy
import sys

from dynamic_reconfigure.srv import *
from dynamic_reconfigure.msg import *
# from dynamic_reconfigure_example.cfg import ExampleConfig

count = 0
hangup = False
die = False

def handle_reconfigure(req):
    rospy.loginfo(req)
    resp = ReconfigureResponse()
    return resp

def update(msg):
    rospy.loginfo(msg)

if __name__ == "__main__":
    rospy.init_node('example_server_manual', anonymous = True)

    ns = rospy.get_namespace() + '/' + rospy.get_name()
    ns = ns.replace('///','/')

    rospy.loginfo('namespace: ' + ns)
    pub = rospy.Publisher(ns + '/parameter_descriptions', ConfigDescription, queue_size=1, latch=True)
    cd = ConfigDescription()
    bool_param = BoolParameter()
    bool_param.name = 'test_bool'
    bool_param.value = True
    cd.dflt.bools.append(bool_param)
    cd.max.bools.append(bool_param)
    bool_param.value = False
    cd.min.bools.append(bool_param)

    group = Group()
    group.name = 'Default'
    param = ParamDescription()
    param.name = bool_param.name
    param.level = 1
    param.type = 'bool'
    param.description = 'test bool'
    group.parameters.append(param)
    cd.groups.append(group)

    pub.publish(cd)

    sub = rospy.Subscriber(ns + '/parameter_updates', Config, update, queue_size=1)

    srv = rospy.Service(ns + '/set_parameters', Reconfigure, handle_reconfigure)

    rospy.spin()
