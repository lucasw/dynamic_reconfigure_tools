# connect to a single dynamic reconfigure server
import os
import rospkg
import rospy

from dynamic_reconfigure.client import Client
from functools import partial
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
# TODO(lucasw) need a library version detection to switch between these?
# ImportError: cannot import name QCheckBox
# from python_qt_binding.QtGui import QCheckBox, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QVBoxLayout, QSlider, QWidget
# this works in qt5 kinetic
from python_qt_binding.QtWidgets import QCheckBox, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QVBoxLayout, QSlider, QWidget
from python_qt_binding import QtCore
from std_msgs.msg import Int32

class DrSingle(Plugin):
    do_update_description = QtCore.pyqtSignal(list)

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
        self.parent_layout = self._widget.findChild(QVBoxLayout, 'vertical_layout')
        self.layout = QGridLayout()
        self.parent_layout.addLayout(self.layout)
        self.val_label = {}
        self.changed_value = {}
        self.widget = {}
        self.do_update_description.connect(self.update_description)
        self.div = 100.0

        self.server_name = rospy.get_param("~server", "tbd")
        # Need to put this is timered callback
        self.client = None
        try:
            self.client = Client(self.server_name, timeout=1,
                                 config_callback=self.config_callback,
                                 description_callback=self.description_callback)
        except:  # ROSException:
            pass

        self.update_timer = rospy.Timer(rospy.Duration(0.05), self.update_configuration)

    def description_callback(self, description):
        # self.description = description
        self.do_update_description.emit(description)

    def update_description(self, description):
        # clear the layout
        for i in reversed(range(self.layout.count())):
            self.layout.itemAt(i).widget().setParent(None)
        self.widget = {}
        # TODO(lucasw) this has the min and max values and types from which to 
        # generate the gui
        # rospy.loginfo(description)
        row = 0
        for param in description:
            rospy.loginfo(param['name'] + " " + str(param['min']) + " " +
                          str(param['max']) + " " + str(param['type']))

            label = QLabel()
            label.setText(param['name'])
            self.layout.addWidget(label, row, 0)

            widget = None
            if param['type'] == 'str':
                widget = QLineEdit()
                widget.setText(param['default'])
                # TODO(lucasw) can 'enter' key be made to resend textChanged?
                widget.textChanged.connect(partial(self.value_changed, param['name']))
                widget.returnPressed.connect(partial(self.text_resend, param['name']))
            elif param['type'] == 'bool':
                widget = QCheckBox()
                widget.setChecked(param['default'])
                self.layout.addWidget(widget, row, 1)
                widget.toggled.connect(partial(self.value_changed, param['name']))
            elif param['type'] == 'double':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                widget = QSlider()
                widget.setValue(param['default'])
                widget.setOrientation(QtCore.Qt.Horizontal)
                widget.setMinimum((param['min']) * self.div)
                widget.setMaximum((param['max']) * self.div)
                widget.valueChanged.connect(partial(self.value_changed, param['name'],
                                            use_div=True))
            elif param['type'] == 'int':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                widget = QSlider()
                widget.setValue(param['default'])
                widget.setOrientation(QtCore.Qt.Horizontal)
                widget.setMinimum((param['min']))
                widget.setMaximum((param['max']))
                widget.valueChanged.connect(partial(self.value_changed, param['name']))
            self.layout.addWidget(widget, row, 1)
            self.widget[param['name']] = widget
            val_label = QLabel()
            val_label.setFixedWidth(100)
            # val_label.setText("0")
            self.layout.addWidget(val_label, row, 2)
            self.val_label[param['name']] = val_label
            row += 1

    def config_callback(self, config):
        # rospy.loginfo(config)
        for param_name in config.keys():
            if param_name in self.val_label.keys():
                self.val_label[param_name].setText(str(config[param_name]))

    def text_resend(self, name):
        self.changed_value[name] = self.widget[name].text()

    def value_changed(self, name, value, use_div=False):
        # TODO(lucasw) also want a periodic update mode
        if use_div:
            value /= self.div
        self.changed_value[name] = value

    def update_configuration(self, evt):
        if len(self.changed_value.keys()) > 0:
            self.client.update_configuration(self.changed_value)
            self.changed_value = {}

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
