#!/usr/bin/env python
# Lucas Walter
# March 2018
# Create a ddynamic reconfigure server from a provided topic type

import roslib.message
import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class DDRTopics():
    def __init__(self):
        self.config = None

        self.msg_name = rospy.get_param("~msg_type", "std_msgs/Float64")
        self.msg_class = roslib.message.get_message_class(self.msg_name)
        rospy.loginfo("{} -> {}".format(self.msg_name, self.msg_class))
        if self.msg_class is None:
            raise Exception("could not load message type {}".format(self.msg_name))

        # TODO(lucasw) no per-topic min/max/defaults, just these for all of them
        default_value = rospy.get_param("~default", 0.0)
        min_value = rospy.get_param("~min", -1.0)
        max_value = rospy.get_param("~max", 1.0)
        max_len = rospy.get_param("~max_len", 30)

        self.ddr = DDynamicReconfigure("")

        self.pubs = {}
        topics = rospy.get_param("~topics", ["out"])
        for topic in topics:
            # TODO(lucasw) this isn't ideal but need to pare down the names
            name = topic.replace("/", "_").lstrip("_")[-max_len:]
            rospy.loginfo("{} -> {}".format(topic, name))
            self.pubs[name] = rospy.Publisher(topic, self.msg_class, queue_size=3)
            if type(default_value) == bool:
                self.ddr.add_variable(name, topic, default_value)
            else:
                self.ddr.add_variable(name, topic, default_value, min_value, max_value)

        self.ddr.start(self.dr_callback)

    def dr_callback(self, config, level):
        if self.config is not None:
            for topic in self.pubs.keys():
                old_value = getattr(self.config, topic)
                new_value = getattr(config, topic)
                if new_value != old_value:
                    # rospy.loginfo("{} {}".format(topic, new_value))
                    # TODO(lucasw) this will only work for a few simple message types
                    self.pubs[topic].publish(self.msg_class(new_value))

        self.config = config
        return config


if __name__ == "__main__":
    rospy.init_node("ddr_topics")
    ddr_topics = DDRTopics()
    rospy.spin()
