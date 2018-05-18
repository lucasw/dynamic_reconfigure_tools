#!/usr/bin/env python

import dynamic_reconfigure.client
import rospy


class ExampleClient():
    def __init__(self):
        rospy.wait_for_service("example_server")
        self.config = None
        self.dr_client = dynamic_reconfigure.client.Client("example_server",
                                                           timeout=4,
                                                           config_callback=self.dr_callback)
        rospy.loginfo("set up server connection")

    def dr_callback(self, config):
        self.config = config
        rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
                      {str_param}, {bool_param}, {enum_param}""".format(**config))


if __name__ == '__main__':
    rospy.init_node("example_client")
    rospy.spin()
