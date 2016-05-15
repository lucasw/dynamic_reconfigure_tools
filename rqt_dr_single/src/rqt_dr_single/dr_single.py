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

    def description_callback(self, description):
        # self.description = description
        self.do_update_description.emit(description)

    def update_description(self, description):
        # clear the layout
        for i in reversed(range(self.layout.count())):
            self.layout.itemAt(i).widget().setParent(None)
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

            if param['type'] == 'str':
                pass
            elif param['type'] == 'bool':
                checkbox = QCheckBox()
                self.layout.addWidget(checkbox, row, 1)
                checkbox.toggled.connect(partial(self.value_changed, param['name']))
            elif param['type'] == 'double':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                slider = QSlider()
                slider.setOrientation(QtCore.Qt.Horizontal)
                slider.setMinimum((param['min']) * self.div)
                slider.setMaximum((param['max']) * self.div)
                self.layout.addWidget(slider, row, 1)
                slider.valueChanged.connect(partial(self.value_changed, param['name'],
                                            use_div=True))
            elif param['type'] == 'int':
                # TODO(lucasw) also have qspinbox or qdoublespinbox
                slider = QSlider()
                slider.setOrientation(QtCore.Qt.Horizontal)
                slider.setMinimum((param['min']))
                slider.setMaximum((param['max']))
                self.layout.addWidget(slider, row, 1)
                slider.valueChanged.connect(partial(self.value_changed, param['name']))
            val_label = QLabel()
            # val_label.setFixedWidth(100)
            # val_label.setText("0")
            self.layout.addWidget(val_label, row, 2)
            self.val_label[param['name']] = val_label
            row += 1

    def config_callback(self, config):
        # rospy.loginfo(config)
        for param_name in config.keys():
            if param_name in self.val_label.keys():
                self.val_label[param_name].setText(str(config[param_name]))

    def value_changed(self, name, value, use_div=False):
        # TODO(lucasw) also want a periodic update mode
        if use_div:
            value /= self.div
        self.client.update_configuration({name: value})

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
