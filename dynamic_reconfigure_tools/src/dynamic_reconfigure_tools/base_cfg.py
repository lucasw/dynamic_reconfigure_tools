'''
example parameters:

{'srcline': 280,
'description': 'x offset',
'max': 2048,
'cconsttype': 'const int',
'ctype': 'int',
'srcfile': '/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py',
'name': 'x_offset',
'edit_method': '',
'default': 0,
'level': 1,
'min': 0,
'type': 'int'},
{'srcline': 280,
'description': 'updates per second',
'max': 50.0,
'cconsttype': 'const double',
'ctype': 'double',
'srcfile': '/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py',
'name': 'update_rate',
'edit_method': '',
'default': 5.0,
'level': 2,
'min': 0.0,
'type': 'double'}
'''
# do a deep copy of this
example_parameter = {
'description': 'tbd',
'name': 'tbd',
'cconsttype': 'const int',  # or const double, bool, str
'ctype': 'int',
'type': 'int',
'default': 0,
'min': 0,
'max': 2048,
'level': 1,
'edit_method': '',
'srcfile': '/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py',
'srcline': 280
}

# TODO(lucasw) support groups
config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 235, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

