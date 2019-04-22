#!/usr/bin/env python

import rospy
import sys

from dynamic_reconfigure.srv import *
from dynamic_reconfigure.msg import *
# from dynamic_reconfigure_example.cfg import ExampleConfig

class ManualDr:
    def __init__(self):
        rospy.init_node('example_server_manual', anonymous = True)

        ns = rospy.get_namespace() + '/' + rospy.get_name()
        ns = ns.replace('///','/')

        rospy.loginfo('namespace: ' + ns)
        self.pub = rospy.Publisher(ns + '/parameter_descriptions', ConfigDescription, queue_size=1, latch=True)
        self.cd = ConfigDescription()
        for name in ['test_bool', 'test_bool2']:
            bool_param = BoolParameter()
            bool_param.name = name
            bool_param.value = True
            self.cd.dflt.bools.append(bool_param)
            self.cd.max.bools.append(bool_param)
            bool_param.value = False
            self.cd.min.bools.append(bool_param)

        group = Group()
        group.name = 'Default'
        group.parent = 0
        group.id = 0
        param = ParamDescription()
        param.name = bool_param.name
        param.level = 1
        param.type = 'bool'
        param.description = 'test bool'
        group.parameters.append(param)
        self.cd.groups.append(group)

        group = Group()
        group.name = 'Test'
        group.parent = 0
        group.id = 1
        param = ParamDescription()
        param.name = bool_param.name
        param.level = 1
        param.type = 'bool'
        param.description = 'test bool2'
        group.parameters.append(param)
        self.cd.groups.append(group)

        self.pub.publish(self.cd)

        self.sub = rospy.Subscriber(ns + '/parameter_updates', Config, self.update, queue_size=1)
        self.srv = rospy.Service(ns + '/set_parameters', Reconfigure, self.handle_reconfigure)

    def handle_reconfigure(self, req):
        rospy.loginfo(req)
        resp = ReconfigureResponse()
        return resp

    def update(msg):
        rospy.loginfo(msg)

if __name__ == "__main__":
    manual_dr = ManualDr()
    rospy.spin()
