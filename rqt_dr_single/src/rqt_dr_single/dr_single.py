import os
import rospkg
import rospy

from dynamic_reconfigure.client import Client
from functools import partial
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QCheckBox, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QSlider, QWidget
from python_qt_binding import QtCore
from std_msgs.msg import Int32

class DrSingle(Plugin):

    def __init__(self, context):
        super(DrSingle, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DrSingle')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self.rospack = rospkg.RosPack()

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(self.rospack.get_path('rqt_dr_single'), 'resource', 'dr_single.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DrSingleUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.server_name = rospy.get_param("~server", "tbd")
        # Need to put this is timered callback
        self.client = None
        try:
            self.client = Client(self.server_name, timeout=1,
                                 config_callback=self.config_callback,
                                 description_callback=self.description_callback)
        except:  # ROSException:
            pass

    def config_callback(self, config):
        # rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))
        rospy.loginfo(config)

    def description_callback(self, description):
        # TODO(lucasw) this has the min and max values and types from which to 
        # generate the gui

        rospy.loginfo(description)
    def temp(self):
        # need to make this a grid instead
        self.parent_layout = self._widget.findChild(QVBoxLayout, 'vertical_layout')
        self.layout = QGridLayout()
        self.parent_layout.addLayout(self.layout)

        self.pubs = {}
        self.subs = {}
        self.val_labels = {}
        #rospy.loginfo(rospy.get_namespace())
        # TODO(lucasw) maybe this should be a pickled string instead
        # of a bunch of params?
        all_params = rospy.get_param_names()
        prefix = rospy.get_namespace() + "controls/"
        prefix_feedback = rospy.get_namespace() + "feedback/"
        row = 0
        for param in all_params:
            if param.find(prefix) >= 0:
                if param.find("_min") < 0 and param.find("_max") < 0 and \
                        param.find("_type") < 0:
                    # TODO

                    param = param.replace(prefix, "")
                    label = QLabel()
                    label.setText(param)
                    self.layout.addWidget(label, row, 0)
                    minimum = rospy.get_param(prefix + param + "_min")
                    maximum = rospy.get_param(prefix + param + "_max")
                    ctrl_type = rospy.get_param(prefix + param + "_type")
                    rospy.loginfo(param + " " + str(minimum) + " " +
                                  str(maximum) + " " + str(ctrl_type))

                    if ctrl_type == 'bool':
                        checkbox = QCheckBox()
                        self.layout.addWidget(checkbox, row, 1)
                        checkbox.toggled.connect(partial(self.publish_value, param))
                    else:  # if ctrl_type == 'int':
                        slider = QSlider()
                        slider.setOrientation(QtCore.Qt.Horizontal)
                        slider.setMinimum(minimum)
                        slider.setMaximum(maximum)
                        self.layout.addWidget(slider, row, 1)
                        slider.valueChanged.connect(partial(self.publish_value, param))
                    val_label = QLabel()
                    val_label.setFixedWidth(50)
                    # val_label.setText("0")
                    self.layout.addWidget(val_label, row, 2)
                    self.val_labels[param] = val_label
                    self.subs[param] = rospy.Subscriber(prefix_feedback + param,
                            Int32, self.feedback_callback, param, queue_size=2)
                    self.pubs[param] = rospy.Publisher(prefix + param,
                            Int32, queue_size=2)
                    row += 1

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
