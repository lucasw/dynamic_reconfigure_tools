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
from python_qt_binding.QtWidgets import QCheckBox, QComboBox, QGridLayout
from python_qt_binding.QtWidgets import QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QSlider, QWidget
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
        self.changed_value = {}
        self.reset()
        self.do_update_description.connect(self.update_description)
        self.div = 100.0

        self.server_name = rospy.get_param("~server", "tbd")
        if len(self.server_name) == 0 or self.server_name[0] != '/':
            self.server_name = rospy.get_namespace() + self.server_name

        self.refresh_button = self._widget.findChild(QPushButton, 'refresh_button')
        self.refresh_button.pressed.connect(self.update_topic_list)

        self.connected_checkbox = self._widget.findChild(QCheckBox, 'connected_checkbox')
        self.connected_checkbox.setChecked(False)
        self.connected_checkbox.setEnabled(False)
        self.server_combobox = self._widget.findChild(QComboBox, 'server_combobox')
        self.server_combobox.currentIndexChanged.connect(self.server_changed)
        self.update_topic_list()

        # Need to put this is timered callback
        self.client = None
        self.update_timer = rospy.Timer(rospy.Duration(0.05), self.update_configuration)

    def update_topic_list(self):
        topics = rospy.get_published_topics()
        self.server_combobox.currentIndexChanged.disconnect(self.server_changed)
        self.server_combobox.clear()
        rospy.loginfo(self.server_name)
        self.server_combobox.addItem(self.server_name)
        dr_list = []
        for topic in topics:
            if topic[1] == 'dynamic_reconfigure/ConfigDescription':
                server_name = topic[0][:topic[0].rfind('/')]
                if server_name != self.server_name:
                    dr_list.append(server_name)
        self.server_combobox.addItems(dr_list)
        self.server_combobox.currentIndexChanged.connect(self.server_changed)

    def server_changed(self, index):
        new_server = self.server_combobox.currentText()
        rospy.loginfo(new_server)
        if self.server_name != new_server:
            self.server_name = new_server
            self.client = None
            self.changed_value = {}
            self.reset()

    def description_callback(self, description):
        # self.description = description
        self.do_update_description.emit(description)

    def reset(self):
        self.described = False
        self.config = None
        self.widget = {}
        self.connections = {}
        self.use_div = {}
        self.val_label = {}

    def update_description(self, description):
        # clear the layout
        for i in reversed(range(self.layout.count())):
            self.layout.itemAt(i).widget().setParent(None)
        # TODO(lucasw) this has the min and max values and types from which to 
        # generate the gui
        # rospy.loginfo(description)
        row = 0
        for param in description:
            rospy.logdebug(param['name'] + " " + str(param['min']) + " " +
                           str(param['max']) + " " + str(param['type']))

            label = QLabel()
            label.setText(param['name'])
            self.layout.addWidget(label, row, 0)

            widget = None
            if param['type'] == 'str':
                widget = QLineEdit()
                widget.setText(param['default'])
                widget.editingFinished.connect(partial(self.text_resend, param['name']))
            elif param['type'] == 'bool':
                widget = QCheckBox()
                widget.setChecked(param['default'])
                self.layout.addWidget(widget, row, 1)
                self.use_div[param['name']] = False
                self.connections[param['name']] = partial(self.value_changed, param['name'])
                widget.toggled.connect(self.connections[param['name']])
            elif param['type'] == 'double':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                widget = QSlider()
                widget.setValue(param['default'])
                widget.setOrientation(QtCore.Qt.Horizontal)
                widget.setMinimum((param['min']) * self.div)
                widget.setMaximum((param['max']) * self.div)
                self.use_div[param['name']] = True
                self.connections[param['name']] = partial(self.value_changed, param['name'],
                                            use_div=True)
                widget.valueChanged.connect(self.connections[param['name']])
            elif param['type'] == 'int':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                widget = QSlider()
                widget.setValue(param['default'])
                widget.setOrientation(QtCore.Qt.Horizontal)
                widget.setMinimum((param['min']))
                widget.setMaximum((param['max']))
                self.use_div[param['name']] = False
                self.connections[param['name']] = partial(self.value_changed, param['name'])
                widget.valueChanged.connect(self.connections[param['name']])
            self.layout.addWidget(widget, row, 1)
            self.widget[param['name']] = widget
            val_label = QLabel()
            val_label.setFixedWidth(100)
            # val_label.setText("0")
            self.layout.addWidget(val_label, row, 2)
            self.val_label[param['name']] = val_label
            row += 1
        self.described = True
        self.update_config()

    def config_callback(self, config):
        # The first config/description callback happen out of order-
        # the description is updated after the config, so need to store it.
        self.config = config
        if self.described:
            self.update_config()

    def update_config(self):
        if not self.config:
            return
        # if not self.client:
        #     return
        # rospy.loginfo(config)
        for param_name in self.config.keys():
            if param_name in self.val_label.keys() and param_name in self.widget.keys():
                self.val_label[param_name].setText(str(self.config[param_name]))
                # TODO(lucasw) also need to change slider
                value = self.config[param_name]
                if type(self.widget[param_name]) is type(QSlider()):
                    self.widget[param_name].valueChanged.disconnect()
                    if self.use_div[param_name]:
                        value = value * self.div
                    self.widget[param_name].setValue(value)
                    self.widget[param_name].valueChanged.connect(self.connections[param_name])
                elif type(self.widget[param_name]) is type(QLineEdit()):
                    self.widget[param_name].setText(value)
                elif type(self.widget[param_name]) is type(QCheckBox()):
                    self.widget[param_name].setChecked(value)
        self.config = None

    def text_resend(self, name):
        self.changed_value[name] = self.widget[name].text()

    def value_changed(self, name, value, use_div=False):
        # TODO(lucasw) also want a periodic update mode
        if use_div:
            value /= self.div
        self.changed_value[name] = value

    def update_configuration(self, evt):
        if self.client is None:
            try:
                self.client = Client(self.server_name, timeout=0.1,
                                     config_callback=self.config_callback,
                                     description_callback=self.description_callback)
            except:  # ROSException:
                return
        self.connected_checkbox.setChecked(True)

        if len(self.changed_value.keys()) > 0:
            try:
                self.client.update_configuration(self.changed_value)
            except:
                rospy.logerr("lost connection to server")
                self.client = None
                self.connected_checkbox.setChecked(False)
                return
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
