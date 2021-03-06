#!/usr/bin/env python
# Lucas Walter
# April 2016
# Create a dynamic reconfigure server dynamically from rosparams
# which will hook them up to topics.
# For v4l2ucp it would be nice to be able to do this in C++
# or alternatively do the v4l2 calls in python.

import copy
import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconfigure_tools import base_cfg
from std_msgs.msg import Empty, Float64, Int32


class DrTopics():
    def __init__(self):
        self.dr_server = None
        self.delta = None
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)
        self.configured_sub = rospy.Subscriber("configured", Empty,
                                               self.config, queue_size=1)
        # TODO(lucasw) this might run concurrently with callback,
        # which results in errors
        # if rospy.get_param("~config_on_init", False):
        #    self.config()

    def config(self, msg=None):
        self.pubs = {}
        self.subs = {}
        self.values = {}
        self.parameters = {}

        # print(dir(base_cfg))
        base_cfg.all_level = 1
        # rospy.loginfo(rospy.get_namespace())
        # TODO(lucasw) maybe this should be a pickled string instead
        # of a bunch of params?
        all_params = rospy.get_param_names()
        prefix = rospy.get_namespace() + "controls/"
        prefix_feedback = rospy.get_namespace() + "feedback/"
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
                    topic = rospy.get_param(prefix + param + "/topic")
                    base_cfg.min[param] = rospy.get_param(prefix + param + "/min")
                    base_cfg.max[param] = rospy.get_param(prefix + param + "/max")
                    # TODO(lucasw) set the default somewhere
                    print(name, ':', prefix + param + "/default")
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
                        self.pubs[param] = rospy.Publisher(topic,
                                                           Int32, queue_size=2)
                    elif base_type == 'double':
                        self.pubs[param] = rospy.Publisher(topic,
                                                           Float64, queue_size=2)
        # TODO(lucasw) if no params are found raise a warning

        self.base_cfg = base_cfg

        # TODO(lucasw) if dr_server is already running from previous
        # init, how to stop it?
        self.dr_server = Server(base_cfg, self.dr_callback)

        # can't create subscribers until dr server is running
        for param in self.values.keys():
            if self.parameters[param]['type'] == 'int':
                self.subs[param] = rospy.Subscriber(prefix_feedback + param,
                                                    Int32, self.feedback_callback,
                                                    param, queue_size=2)
            elif self.parameters[param]['type'] == 'double':
                self.subs[param] = rospy.Subscriber(prefix_feedback + param,
                                                    Float64, self.feedback_callback,
                                                    param, queue_size=2)

    def dr_callback(self, config, level):
        for key in config.groups.parameters.keys():
            if level & self.base_cfg.level[key]:
                if self.parameters[key]['type'] == 'int':
                    self.pubs[key].publish(Int32(config[key]))
                elif self.parameters[key]['type'] == 'double':
                    self.pubs[key].publish(Float64(config[key]))
        return config

    def feedback_callback(self, msg, param):
        self.values[param] = msg.data
        # dict update the dr server
        self.delta = {}
        self.delta[param] = msg.data

    def update(self, evt):
        if self.delta:
            self.dr_server.update_configuration(self.delta)
            self.delta = None


if __name__ == "__main__":
    rospy.init_node("dr_topics")
    dr_topics = DrTopics()
    rospy.spin()
