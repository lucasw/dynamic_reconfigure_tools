#!/usr/bin/env python
# Lucas Walter
# May 2016
# Create a dynamic reconfigure server dynamically from rosparams
# which will hook them up to other dynamic reconfigure servers.
# This allows a static dr panel of controls with only a subset
# of all available controls.

# roslaunch vimjay dev0_stop_motion.launch
# rosparam delete /test/controls
# rosparam load stop_motion_dr.yaml /test/controls
# rosrun dynamic_reconfigure_tools dr2dr.py __ns:=/test

# each control is a mixed list
# [server name, server value name, type, level, description, default value, min, max]
# TODO support enum and hierarchy late later

import copy
import dynamic_reconfigure
import rospy

# from dynamic_reconfigure.parameter_generator_catkin import *
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from functools import partial
from dynamic_reconfigure_tools import base_cfg
from std_msgs.msg import Empty, Int32


class Dr2Dr():
    def __init__(self):
        wait_for_config = rospy.get_param("~wait_for_config", False)
        if wait_for_config:
            self.configured_sub = rospy.Subscriber("configured", Empty,
                                                   self.config, queue_size=1)
        else:
            self.config(None)

    def config(self, msg):
        self.values = {}
        self.client = {}
        self.client_of_param = {}
        self.server_params = {}
        self.server_value_name = {}

        # print dir(base_cfg)
        base_cfg.all_level = 1
        # rospy.loginfo(rospy.get_namespace())
        all_params = rospy.get_param_names()
        prefix = rospy.get_namespace() + "controls/"
        rospy.logdebug(rospy.get_namespace())
        for param in all_params:
            if param[0:len(prefix)] == prefix:

                config = rospy.get_param(param)
                param = param.replace(prefix, "")
                rospy.logdebug(param)
                # rospy.loginfo(config)
                # self.server[param] = config[0]
                server_name = config[0]
                if server_name not in self.server_params.keys():
                    self.server_params[server_name] = []
                self.server_params[server_name].append(param)
                rospy.loginfo("new param: " + server_name + " " + param + " " +
                              str(self.server_params[server_name]))

                self.server_value_name[param] = config[1]
                base_cfg.type[param] = config[2]
                base_cfg.level[param] = config[3]
                description = config[4]
                base_cfg.defaults[param] = config[5]
                base_cfg.min[param] = config[6]
                base_cfg.max[param] = config[7]

                parameter = copy.deepcopy(base_cfg.example_parameter)
                parameter['name'] = param
                parameter['cconst type'] = 'const ' + base_cfg.type[param]
                parameter['ctype'] = base_cfg.type[param]
                parameter['type'] = base_cfg.type[param]
                parameter['description'] = description
                parameter['default'] = base_cfg.defaults[param]
                parameter['min'] = base_cfg.min[param]
                parameter['max'] = base_cfg.max[param]
                parameter['level'] = base_cfg.level[param]
                base_cfg.config_description['parameters'].append(parameter)
        # TODO(lucasw) if no params are found raise a warning

        self.configured = False
        self.break_feedback = False
        self.break_feedback2 = False

        self.base_cfg = base_cfg
        self.dr_server = Server(base_cfg, self.dr_callback)

        rospy.loginfo(self.server_params)
        for server_name in self.server_params.keys():
            params = self.server_params[server_name]
            # rospy.loginfo(server_name + " " + str(params))
            self.client[server_name] = None
            try:
                self.client[server_name] = Client(server_name, timeout=1,
                                                  config_callback=partial(self.upstream_dr_callback,
                                                                          params))
            except rospy.exceptions.ROSException as e:
                # TODO(lucasw) if the server doesn't exist yet, retry later
                rospy.logwarn(e)

            for param in params:
                rospy.loginfo(param + " " + server_name)
                self.client_of_param[param] = self.client[server_name]
        self.configured = True

    # the callback from this server update the other servers through the clients
    def dr_callback(self, config, level):
        # print level, config, self.configured, self.break_feedback
        if not self.configured:
            return config
        if self.break_feedback:
            return config
        for key in config.groups.parameters.keys():
            if level & self.base_cfg.level[key]:
                server_key_name = self.server_value_name[key]
                value = config[key]
                # TODO(lucasw) conglomerate updates from the same server
                if self.client_of_param[key]:
                    self.break_feedback2 = True
                    try:
                        self.client_of_param[key].update_configuration({server_key_name: value})
                    except dynamic_reconfigure.DynamicReconfigureParameterException as e:
                        rospy.loginfo(str(level) + " " + key + ": " + str(value))
                        rospy.logwarn(e)
                    self.break_feedback2 = False
                # TODO(lucasw) else try creating the client again- or
                # have timered update that tries that at regular rate?
        return config

    # the callback from the all the upstream servers updates the local server
    # TODO(lucasw) thought config would come before params from partial,
    # but that is not the case.
    def upstream_dr_callback(self, params, config):
        if self.break_feedback2:
            return
        # rospy.loginfo('config: ' + str(config))
        # rospy.loginfo('params: ' + str(params))
        updates = {}
        for param in params:
            updates[param] = config[self.server_value_name[param]]
        # rospy.loginfo(updates)
        # Update the local values, which will in turn trigger dr_callback
        self.break_feedback = True
        self.dr_server.update_configuration(updates)
        self.break_feedback = False
        # http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient


if __name__ == "__main__":
    rospy.init_node("dr2dr")
    dr2dr = Dr2Dr()
    rospy.spin()
