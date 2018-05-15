# connect to a single dynamic reconfigure server
import os
import rospkg
import rospy
import time

from dynamic_reconfigure.client import Client
from functools import partial
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
# TODO(lucasw) need a library version detection to switch between these?
# ImportError: cannot import name QCheckBox
# from python_qt_binding.QtGui import QCheckBox, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QVBoxLayout, QSlider, QWidget
# this works in qt5 kinetic
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtGui import QDoubleValidator, QIntValidator
from python_qt_binding.QtWidgets import QCheckBox, QComboBox, QGridLayout
from python_qt_binding.QtWidgets import QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QSlider, QWidget

from python_qt_binding import QtCore
from std_msgs.msg import Int32

class DrSingle(Plugin):
    do_update_description = QtCore.pyqtSignal(list)
    do_update_config = QtCore.pyqtSignal(dict)
    do_update_checkbox = QtCore.pyqtSignal(bool)
    do_update_dr = QtCore.pyqtSignal()

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
        # self.parent_layout = self._widget.findChild(QVBoxLayout, 'vertical_layout')
        self.layout = self._widget.findChild(QGridLayout, 'grid_layout')
        self.changed_value = {}
        self.reset()
        self.do_update_description.connect(self.update_description)
        self.do_update_config.connect(self.update_config)
        self.do_update_checkbox.connect(self.update_checkbox)
        self.do_update_dr.connect(self.update_dr)
        self.div = 100.0

        self.server_name = rospy.get_param("~server", None)
        if self.server_name is not None and self.server_name[0] != '/':
            self.server_name = rospy.get_namespace() + self.server_name

        self.hide_dropdown = rospy.get_param("~hide_dropdown", None)

        self.refresh_button = self._widget.findChild(QPushButton, 'refresh_button')
        self.refresh_button.pressed.connect(self.update_topic_list)

        self.connected_checkbox = self._widget.findChild(QCheckBox, 'connected_checkbox')
        self.connected_checkbox.setChecked(False)
        self.connected_checkbox.setEnabled(False)
        self.server_combobox = self._widget.findChild(QComboBox, 'server_combobox')
        self.server_combobox.currentIndexChanged.connect(self.server_changed)
        self.client = None
        self.update_topic_list()

        # try to connect to saved dr server
        self.connect_dr()
        # TODO(lucasw) can't use ros timers in guis, there might be a sim clock that
        # is paused.  (How many other nodes in other projects are going to fail
        # because of that?)
        # self.update_timer = rospy.Timer(rospy.Duration(0.05), self.update_dr_configuration)
        # TODO(lucasw) is this the right thread to be calling this?
        # If this is qt then need to trigger a ros callback-
        # can I do a single shot rospy.Timer with sim_clock paused?
        # self.do_update_dr.emit()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_dr_from_emit)
        self.timer.start(100)

    def update_dr_from_emit(self):
        self.do_update_dr.emit()

    def update_dr(self):
        self.update_dr_configuration(None)
        # TODO(lucasw) need to call this repeatedly, setup time callback here

    def update_topic_list(self):
        # TODO(lucasw) if the roscore goes down, this throws a socket error
        topics = rospy.get_published_topics()
        self.server_combobox.currentIndexChanged.disconnect(self.server_changed)
        self.server_combobox.clear()
        rospy.logdebug(self.server_name)
        if self.server_name is not None:
            self.server_combobox.addItem(self.server_name)
        dr_list = []
        for topic in topics:
            if topic[1] == 'dynamic_reconfigure/ConfigDescription':
                server_name = topic[0][:topic[0].rfind('/')]
                if server_name != self.server_name:
                    dr_list.append(server_name)
                # default to choosing the first one
                # if self.server_name is None:
                #     self.server_name = server_name
        self.server_combobox.addItems(dr_list)
        self.server_combobox.currentIndexChanged.connect(self.server_changed)

        # force the gui to be refreshed
        # self.client = None
        if self.client is None:
            self.connect_dr()
        self.update_dr_configuration(None)

    def server_changed(self, index):
        new_server = self.server_combobox.currentText()
        rospy.loginfo(new_server)
        if self.server_name != new_server:
            self.server_name = new_server
            self.client = None
            self.changed_value = {}
            self.reset()
            self.refresh_button.click()

    def description_callback(self, description):
        # self.description = description
        self.do_update_description.emit(description)

    def reset(self):
        rospy.logdebug("reset")
        self.described = False
        self.widget = {}
        self.enum_values = {}
        self.enum_inds = {}
        self.connections = {}
        self.use_div = {}
        self.val_label = {}
        self.config = None

    def add_label(self, name, row):
        # TODO(lucasw) don't really need this
        return
        val_label = QLabel()
        val_label.setFixedWidth(90)
        self.layout.addWidget(val_label, row, 2)
        self.val_label[name] = val_label

    def make_line_edit(self, name, row, vmin, vmax, double_not_int):
        val_edit = QLineEdit()
        val_edit.setFixedWidth(90)
        # TODO(lucasw) have optional ability to break limits
        if double_not_int:
            val_edit.setValidator(QDoubleValidator(vmin,
                                                   vmax, 8, self))
        else:
            val_edit.setValidator(QIntValidator(vmin,
                                                vmax, self))
        connection_name = name + "_line_edit"
        self.connections[connection_name] = partial(self.text_changed,
                                                    name)
        val_edit.editingFinished.connect(self.connections[connection_name])
        self.layout.addWidget(val_edit, row, 2)
        self.val_label[name] = val_edit
        return val_edit

    # Setup the gui according to the dr description
    def update_description(self, description):
        if rospy.is_shutdown():
            return
        # clear the layout
        # rospy.loginfo("clearing layout " + str(self.layout.count()))
        for i in reversed(range(self.layout.count())):
            layout = self.layout.itemAt(i).layout()
            if layout:
                for j in reversed(range(layout.count())):
                    layout.itemAt(j).widget().setParent(None)
                self.layout.itemAt(i).layout().setParent(None)
            elif self.layout.itemAt(i).widget():
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
                self.add_label(param['name'], row)
                self.layout.addWidget(widget, row, 1)
            elif param['type'] == 'bool':
                widget = QCheckBox()
                widget.setChecked(param['default'])
                self.layout.addWidget(widget, row, 1)
                self.use_div[param['name']] = False
                self.connections[param['name']] = partial(self.value_changed, param['name'])
                widget.toggled.connect(self.connections[param['name']])
                self.add_label(param['name'], row)
                self.layout.addWidget(widget, row, 1)
            elif param['type'] == 'double':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                layout = QHBoxLayout()
                widget = QSlider()
                widget.setValue(param['default'])
                widget.setOrientation(QtCore.Qt.Horizontal)
                widget.setMinimum((param['min']) * self.div)
                widget.setMaximum((param['max']) * self.div)
                self.use_div[param['name']] = True
                self.connections[param['name']] = partial(self.value_changed,
                                                          param['name'],
                                                          use_div=True)
                widget.valueChanged.connect(self.connections[param['name']])
                layout.addWidget(widget)

                line_edit = self.make_line_edit(param['name'], row,
                                                param['min'], param['max'],
                                                double_not_int=True)
                layout.addWidget(line_edit)
                self.layout.addLayout(layout, row, 1)

            elif param['type'] == 'int':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                if param['edit_method'] == '':
                    layout = QHBoxLayout()
                    widget = QSlider()
                    widget.setValue(param['default'])
                    widget.setOrientation(QtCore.Qt.Horizontal)
                    widget.setMinimum((param['min']))
                    widget.setMaximum((param['max']))
                    self.connections[param['name']] = partial(self.value_changed, param['name'])
                    widget.valueChanged.connect(self.connections[param['name']])
                    layout.addWidget(widget)
                    line_edit = self.make_line_edit(param['name'], row,
                                                    param['min'], param['max'],
                                                   double_not_int=False)
                    layout.addWidget(line_edit)
                    self.layout.addLayout(layout, row, 1)
                else:  # enum
                    widget = QComboBox()
                    # edit_method is actually a long string that has to be interpretted
                    # back into a list
                    enums = eval(param['edit_method'])['enum']
                    self.enum_values[param['name']] = {}
                    self.enum_inds[param['name']] = {}
                    count = 0
                    for enum in enums:
                        name = enum['name'] + ' (' + str(enum['value']) + ')'
                        widget.addItem(name)
                        self.enum_values[param['name']][count] = enum['value']
                        self.enum_inds[param['name']][enum['value']] = count
                        count += 1
                        # print count, enum
                    self.connections[param['name']] = partial(self.enum_changed,
                                                              param['name'])
                    widget.currentIndexChanged.connect(self.connections[param['name']])
                    self.add_label(param['name'], row)
                    self.layout.addWidget(widget, row, 1)
                self.use_div[param['name']] = False
            else:
                rospy.logerr(param)

            if widget:
                self.widget[param['name']] = widget
                # val_label.setText("0")
                # self.layout.addWidget(val_label, row, 2)
                row += 1
        self.described = True
        rospy.loginfo("updated description")
        if self.config:
            self.update_config(self.config)

    def config_callback(self, config):
        # The first config/description callback happen out of order-
        # the description is updated after the config, so need to store it.
        self.do_update_config.emit(config)

    def update_config(self, config):
        if not config:
            return
        if not self.described:
            self.config = config
            return
        # if not self.client:
        #     return
        rospy.logdebug(config)
        for param_name in config.keys():
            if param_name in self.widget.keys():
                if param_name in self.val_label.keys():
                    self.val_label[param_name].setText(str(config[param_name]))
                # TODO(lucasw) also need to change slider
                value = config[param_name]
                if type(self.widget[param_name]) is type(QSlider()):
                    try:
                        self.widget[param_name].valueChanged.disconnect()
                    except TypeError as e:
                        # TOOD(lucasw) not sure in what circumstances this fails
                        rospy.logwarn(param_name + " disconnect failed " + str(e))
                        # TODO(lucasw) if that failed will the connect work?
                    if self.use_div[param_name]:
                        value = value * self.div
                    self.widget[param_name].setValue(value)
                    self.widget[param_name].valueChanged.connect(self.connections[param_name])
                elif type(self.widget[param_name]) is type(QLineEdit()):
                    self.widget[param_name].setText(value)
                elif type(self.widget[param_name]) is type(QCheckBox()):
                    self.widget[param_name].setChecked(value)
                elif type(self.widget[param_name]) is type(QComboBox()):
                    self.widget[param_name].setCurrentIndex(self.enum_inds[param_name][value])

    def text_resend(self, name):
        self.changed_value[name] = self.widget[name].text()
        # TODO(lucasw) wanted to avoid these with a timered loop, but doing it direct for now
        # self.do_update_dr.emit()

    def enum_changed(self, name, ind):
        if not ind in self.enum_values[name].keys():
            return
        #     rospy.logerr(name + " values ind mismatch " + str(ind) + " " +
        #                  str(self.enum_values[name].keys()))
        #     return
        self.changed_value[name] = self.enum_values[name][ind]
        # TODO(lucasw) wanted to avoid these with a timered loop, but doing it direct for now
        # self.do_update_dr.emit()

    def text_changed(self, name):
        value = float(self.val_label[name].text())
        self.changed_value[name] = value
        # TODO(lucasw) wanted to avoid these with a timered loop, but doing it direct for now
        # self.do_update_dr.emit()

    def value_changed(self, name, value, use_div=False):
        if use_div:
            value /= self.div
        self.changed_value[name] = value
        # TODO(lucasw) wanted to avoid these with a timered loop, but doing it direct for now
        # self.do_update_dr.emit()

    def update_checkbox(self, value):
        self.connected_checkbox.setChecked(value)

    def connect_dr(self):
        if self.server_name is None:
            return
        try:
            # TODO(lucasw) surely this timeout has nothing to do with ros time
            # and instead uses wall time.
            # This takes up 0.3 second no matter what
            # and seems to block the main gui thread  though I haven't
            # exhausted options for running it in other threads
            self.client = Client(self.server_name, timeout=0.2,
                                 config_callback=self.config_callback,
                                 description_callback=self.description_callback)
            self.do_update_checkbox.emit(True)
        except:  # ROSException:
            rospy.logdebug("no server " + str(self.server_name))

    def update_dr_configuration(self, evt):
        if self.client is None:
            return
        if len(self.changed_value.keys()) > 0:
            try:
                self.client.update_configuration(self.changed_value)
            except:
                rospy.logerr("lost connection to server " + str(self.server_name))
                self.client = None
                self.do_update_checkbox.emit(False)
                return
            self.changed_value = {}

    def shutdown_plugin(self):
        self.reset()
        # TODO unregister all publishers here
        self.timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        rospy.logdebug("saving server " + self.server_name)
        instance_settings.set_value('server_name', self.server_name)
        instance_settings.set_value('hide_dropdown', self.hide_dropdown)
        # goes to ~/.config/ros.org/rqt_gui.ini, or into .perspective

    # This is called after init
    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('server_name') and self.server_name is None:
            self.server_name = instance_settings.value('server_name')
            rospy.logdebug("restore server " + self.server_name)
            self.update_topic_list()
        if self.server_name is None:
            self.server_changed(0)

        if instance_settings.contains('hide_dropdown') and self.hide_dropdown is None:
            # instance settings don't resolve as True of False boolean
            # rospy.loginfo(type(instance_settings.value('hide_dropdown'))) # 'unicode'
            self.hide_dropdown = instance_settings.value('hide_dropdown') == 'true'
        if self.hide_dropdown is None:
            self.hide_dropdown = False
        if self.hide_dropdown:
            self.server_combobox.hide()

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
