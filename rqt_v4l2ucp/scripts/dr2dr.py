#!/usr/bin/env python
# Lucas Walter
# May 2016
# Create a dynamic reconfigure server dynamically from rosparams
# which will hook them up to other dynamic reconfigure servers.
# This allows a static dr panel of controls with only a subset
# of all available controls.

# each control is a mixed list
# [server name, server value name, type, level, description, default value, min, max]
# TODO support enum and hierarchy late later

import copy
import rospy

# from dynamic_reconfigure.parameter_generator_catkin import *
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from functools import partial
from rqt_v4l2ucp import base_cfg
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
        self.server = {}
        self.server_params = {}
        self.server_value_name = {}

        # print dir(base_cfg)
        base_cfg.all_level = 1
        #rospy.loginfo(rospy.get_namespace())
        all_params = rospy.get_param_names()
        prefix = rospy.get_namespace() + "controls/"
        print rospy.get_namespace()
        for param in all_params:
            if param[0:len(prefix)] == prefix:

                param = param.replace(prefix, "")
                config = rospy.get_param(param)

                #self.server[param] = config[0]
                if not config[0] in self.server_params.keys():
                    self.server_params[config[0]] = []
                self.server_params[config[0]].append(param)

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

        for key in self.server_params.keys():
            # TODO(lwalter) if the server doesn't exist yet, retry later
            self.server[key] = Client(key, timeout=1,
                                      config_callback=partial(self.upstream_dr_callback,
                                                              self.server_params[key]))
        # TODO(lucasw) if no params are found raise a warning

        self.base_cfg = base_cfg
        self.dr_server = Server(base_cfg, self.dr_callback)

    # the callback from this one server 
    def dr_callback(self, config, level):
        for key in config.groups.parameters.keys():
            if level & self.base_cfg.level[key]:
                # self.pubs[key].publish(Int32(config[key]))
        return config

    # the callback from the all the other upstream Servers
    def upstream_dr_callback(self, config, params):
        print config
        print params
        # Update the local values
        # http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient
        # look through config names and see which ones match up to params
        # self.server[key].update_configuration({params[0]: config, params[1]: ... })

if __name__ == "__main__":
    rospy.init_node("dr2dr")
    dr2dr = Dr2Dr()
    rospy.spin()
