#!/usr/bin/env python
# Lucas Walter
# March 2018
# Create a dynamic reconfigure server dynamically from a provided topic type

import copy
import roslib.message
import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure_tools import base_cfg
from std_msgs.msg import Empty
# from std_msgs.msg import Float64
# from std_msgs.msg import Int32


class DrTopics():
    def __init__(self):
        self.dr_server = None

        self.msg_name = rospy.get_param("~msg_type", "std_msgs/Float32")
        self.pub = None
        self.configured_sub = rospy.Subscriber("configured", Empty,
                                               self.config, queue_size=1)
        # TODO(lucasw) this might run concurrently with callback,
        # which results in errors
        # if rospy.get_param("~config_on_init", False):
        #    self.config()

        self.dt = rospy.get_param("~dt", 1.0)
        self.msg_class = None
        self.timer = None
        self.make_pub()

    def make_pub(self):
        self.msg_class = roslib.message.get_message_class(self.msg_name)
        rospy.loginfo(f"{self.msg_name} {self.msg_class}, update at {self.dt}s")

    def dont_use(self):
        self.msg_class = roslib.message.get_message_class(self.msg_name)
        all_fields = dir(self.msg_class)
        # generic_fields = dir(rospy.Message)
        # generic_fields = dir(rospy.AnyMsg)
        generic_fields = dir(Empty)

        fields = []
        for field in all_fields:
            if field not in generic_fields:
                fields.append(field)
        print(fields)
        # TODO look in rqt message publisher
        for field in fields:
            print(field, ": ", dir(getattr(self.msg_class(), field)))
        if self.pub:
            self.pub.unregister()
        self.pub = rospy.Publisher("output", self.msg_class, queue_size=5)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update)

    def update(self, event):
        msg = self.msg_class()
        self.pub.publish(msg)

    def config(self, msg=None):
        self.values = {}
        self.parameters = {}

        # print(dir(base_cfg))
        base_cfg.all_level = 1
        # rospy.loginfo(rospy.get_namespace())
        all_params = rospy.get_param_names()
        prefix = rospy.get_namespace() + "controls/"
        # prefix_feedback = rospy.get_namespace() + "feedback/"
        rospy.loginfo(rospy.get_namespace())
        level_shift = 0
        for param in all_params:
            if param[0:len(prefix)] == prefix:
                if param[-len("/name"):] == "/name":
                    # TODO(lucasw) this is more readable than 'param',
                    # but might contain illegal characters
                    name = rospy.get_param(param)
                    param = param.replace(prefix, "")
                    param = param.replace("/name", "")
                    # topic = rospy.get_param(prefix + param + "/topic")
                    base_cfg.min[param] = rospy.get_param(prefix + param + "/min")
                    base_cfg.max[param] = rospy.get_param(prefix + param + "/max")
                    # TODO(lucasw) set the default somewhere
                    print(f"{name}: {prefix}{param}/default")
                    default = rospy.get_param(prefix + param + "/default", base_cfg.min[param])
                    base_cfg.defaults[param] = default
                    self.values[param] = base_cfg.defaults[param]

                    base_type = rospy.get_param(prefix + param + "/type")
                    if base_type == 'menu' or base_type == 'button':
                        base_type = 'int'
                    base_cfg.type[param] = base_type
                    base_cfg.level[param] = 1 << (level_shift % 32)
                    level_shift += 1
                    # rospy.loginfo(param + " " + str(minimum) + " " +
                    #               str(maximum) + " " + str(ctrl_type))
                    parameter = copy.deepcopy(base_cfg.example_parameter)
                    parameter['name'] = param
                    parameter['cconst type'] = 'const ' + base_type
                    parameter['ctype'] = base_type
                    parameter['type'] = base_type
                    parameter['min'] = base_cfg.min[param]
                    parameter['max'] = base_cfg.max[param]
                    parameter['level'] = base_cfg.level[param]
                    self.parameters[param] = parameter
                    base_cfg.config_description['parameters'].append(parameter)
                    # TODO(lucasw) use Float64, Int32 as types
                    if base_type == 'int':
                        pass
                    elif base_type == 'double':
                        pass
        # TODO(lucasw) if no params are found raise a warning

        self.base_cfg = base_cfg

        # TODO(lucasw) if dr_server is already running from previous
        # init, how to stop it?
        self.dr_server = Server(base_cfg, self.dr_callback)

    def dr_callback(self, config, level):
        for key in config.groups.parameters.keys():
            if level & self.base_cfg.level[key]:
                if self.parameters[key]['type'] == 'int':
                    pass
                    # self.pubs[key].publish(Int32(config[key]))
                elif self.parameters[key]['type'] == 'double':
                    pass
                    # self.pubs[key].publish(Float64(config[key]))
        return config

    def feedback_callback(self, msg, param):
        self.values[param] = msg.data
        # dict update the dr server
        delta = {}
        delta[param] = msg.data
        self.dr_server.update_configuration(delta)


if __name__ == "__main__":
    rospy.init_node("dr_topics")
    dr_topics = DrTopics()
    rospy.spin()
