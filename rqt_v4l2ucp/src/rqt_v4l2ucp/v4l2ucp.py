import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QHBoxLayout, QLabel, QVBoxLayout, QSlider, QWidget
from python_qt_binding import QtCore

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

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

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'v4l2ucp.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.layout = self._widget.findChild(QVBoxLayout, 'vertical_layout')

        # TODO need to look through controls namespace params and create
        # control for them, put this in a function and allow refreshing

        #rospy.loginfo(rospy.get_namespace())
        all_params = rospy.get_param_names()
        for param in all_params:
            if param.find(rospy.get_namespace() + "controls/") >= 0:
                if param.find("_min") < 0 and param.find("_max") < 0:
                    # TODO
                    hlayout = QHBoxLayout()
                    self.layout.addLayout(hlayout)

                    rospy.loginfo(param)
                    label = QLabel()
                    label.setText(param)
                    hlayout.addWidget(label)
                    minimum = rospy.get_param(param + "_min")
                    maximum = rospy.get_param(param + "_max")

                    slider = QSlider()
                    slider.setOrientation(QtCore.Qt.Horizontal)
                    hlayout.addWidget(slider)

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
