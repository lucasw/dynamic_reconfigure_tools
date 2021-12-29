#!/usr/bin/env python

import rospy

from dynamic_reconfigure.msg import BoolParameter
from dynamic_reconfigure.msg import Config
from dynamic_reconfigure.msg import ConfigDescription
from dynamic_reconfigure.msg import Group
from dynamic_reconfigure.msg import GroupState
from dynamic_reconfigure.msg import ParamDescription
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.srv import ReconfigureResponse


class ManualDr:
    def __init__(self):
        rospy.init_node('example_server_manual', anonymous=True)

        ns = rospy.get_namespace() + '/' + rospy.get_name()
        ns = ns.replace('///', '/')

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

        group_names = ['Default', 'Test']

        group_state = GroupState()
        group_state.name = group_names[0]
        group_state.state = True
        group_state.parent = 0
        group_state.id = 0
        self.cd.dflt.groups.append(group_state)

        group_state = GroupState()
        group_state.name = group_names[1]
        group_state.state = True
        group_state.parent = 0
        group_state.id = 1
        self.cd.dflt.groups.append(group_state)

        group = Group()
        group.name = group_names[0]
        group.parent = 0
        group.id = 0
        param = ParamDescription()
        param.name = 'test_bool'
        param.level = 1
        param.type = 'bool'
        param.description = 'test bool'
        group.parameters.append(param)
        self.cd.groups.append(group)

        group = Group()
        group.name = group_names[1]
        group.parent = 0
        group.id = 1
        param = ParamDescription()
        param.name = 'test_bool2'
        param.level = 1
        param.type = 'bool'
        param.description = 'test bool2'
        group.parameters.append(param)
        self.cd.groups.append(group)

        self.pub.publish(self.cd)

        self.sub = rospy.Subscriber(ns + '/parameter_updates', Config, self.update, queue_size=1)
        self.update_pub = rospy.Publisher(ns + '/parameter_updates', Config, queue_size=1)
        self.srv = rospy.Service(ns + '/set_parameters', Reconfigure, self.handle_reconfigure)

    def handle_reconfigure(self, req):
        rospy.loginfo(req)

        for b_param in req.config.bools:
            for i in range(len(self.cd.dflt.bools)):
                if b_param.name == self.cd.dflt.bools[i].name:
                    self.cd.dflt.bools[i].value = b_param.value
                    print("new value " + b_param.name + " " + str(self.cd.dflt.bools[i].value))
        # TODO(lucasw)
        # this will get all the updates to every subscriber that has been listening
        # since the beginning, but need all the parameters for new subscribers
        self.update_pub.publish(self.cd.dflt)
        # self.pub.publish(self.cd)

        resp = ReconfigureResponse()
        return resp

    def update(self, msg):
        rospy.loginfo(msg)


if __name__ == "__main__":
    manual_dr = ManualDr()
    rospy.spin()
