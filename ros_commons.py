import importlib

import hypothesis.strategies as st
import numpy as np
import hypothesis.extra.numpy as npst
import re

from ros_basic_strategies import array, string, time, duration
try:
    import rospy
    import rosmsg
    import rospkg
    from rosgraph_msgs.msg import *
    from geometry_msgs.msg import *
    from std_msgs.msg import *
    from sensor_msgs.msg import *
    from diagnostic_msgs.msg import *
    from nav_msgs.msg import *
    from shape_msgs.msg import *
    from stereo_msgs.msg import *
    from trajectory_msgs.msg import *
    from visualization_msgs.msg import *
    from control_msgs.msg import *
    from tf2_msgs.msg import *
    from actionlib_msgs.msg import *
except ImportError:
    print "Please install ROS first"



def ros_msg_loader(msg_type):
    pattern = re.compile('([\w]+)\/([\w]+)')
    match = pattern.search(msg_type)
    module_name = match.group(1) + '.msg'
    class_name = match.group(2)
    module = importlib.import_module(module_name)
    msg_class = module.__dict__[class_name]
    return msg_class


def ros_msg_list():
    msg_list = []
    ros_pack = rospkg.RosPack()
    packs = sorted([x for x in rosmsg.iterate_packages(ros_pack, rosmsg.MODE_MSG)])
    for (p, path) in packs:
        for file in rosmsg.list_types(p):
            msg_list.append(file.split('/')[1])
    return msg_list


def check_msg_type(msg_list, msg_str):
    if msg_str in msg_list:
        return True
    return False


def create_publisher(topic, msg_type):
    pub = rospy.Publisher(topic, msg_type, queue_size=10)
    rospy.init_node('fuzzer_node', anonymous=False)
    return pub


def map_ros_types(type_name):
    strategy_dict = {}
    slot_names = type_name.__slots__
    slot_types = type_name._slot_types
    slots_full = list(zip(slot_names, slot_types))
    for s_name, s_type in slots_full:
        try:
            if '[' and ']' in s_type:
                array_size = s_type[s_type.index('[') + 1:s_type.index(']')]
                if array_size == '':
                    array_size = None  # TODO: not None!
                else:
                    array_size = int(array_size)
                aux = s_type.split('[')[0]
                if aux == 'string':
                    strategy_dict[s_name] = array(elements=string(), min_size=array_size, max_size=array_size)
                else:
                    strategy_dict[s_name] = array(elements=npst.from_dtype(np.dtype(aux)), min_size=array_size,
                                                  max_size=array_size)
            elif s_type is 'string':
                strategy_dict[s_name] = st.text()
            elif s_type is 'time':
                strategy_dict[s_name] = time()
            elif s_type is 'duration':
                strategy_dict[s_name] = duration()
            else:  # numpy compatible ROS built-in types
                strategy_dict[s_name] = npst.from_dtype(np.dtype(s_type))
        except TypeError:
            # TODO: Complex type arrays
            if '/' in s_type and '[]' not in s_type:
                s_type_fix = s_type.split('/')[1]  # e.g. std_msgs/Header take just Header
                strategy_dict[s_name] = map_ros_types(eval(s_type_fix))
            elif '/' in s_type and '[]' in s_type:
                # TODO: Implement complex types fixed value arrays
                s_type_fix = s_type.split('/')[1].split('[')[0]  # e.g. std_msgs/Header take just Header
                strategy_dict[s_name] = array(elements=map_ros_types(eval(s_type_fix)))
    return dynamic_strategy_generator_ros(type_name, strategy_dict)


# A better approach. It returns an instance of a ROS msg directly, so no need for mapping! :)
@st.composite
def dynamic_strategy_generator_ros(draw, type_name, strategy_dict):  # This generates existing ROS msgs objects
    aux_obj = type_name()
    for key, value in strategy_dict.iteritems():
        setattr(aux_obj, key, draw(value))
    return aux_obj


'''
# Calling this.example returns a class with attributes, not an instance! Class.__dict__ returns nothing.
# For returning and instance see:
        # https://www.python-course.eu/python3_classes_and_type.php
        # http://jelly.codes/articles/python-dynamically-creating-classes/
@st.composite
def dynamic_strategy_generator(draw, type_name, strategy_dict):  # This generates unexisting new Objects parallel top ROS msgs
    aux_dict = {}
    for key, value in strategy_dict.iteritems():
        aux_dict[key] = draw(value)
    GenClass = type(type_name.__name__, (), aux_dict)
    return GenClass()


def map_msgs(msg_type, parallel_class):
    class_attrs = [getattr(parallel_class, a) for a in dir(parallel_class) if not a.startswith('__')]
    aux = list(zip(msg_type.__slots__, class_attrs))
    msg_instance = msg_type()  # instance of some msg
    for attr, value in aux:
        setattr(msg_instance, attr, value)
    return msg_instance
'''
