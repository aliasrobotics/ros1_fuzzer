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
except ImportError:
    print "Please install ROS first"


def ros_type_to_dict(msg_type):
    type_regexp = re.compile(r'^(?P<complex>(?P<module>[\w]+)/)?(?P<type>[\w]+)(?P<array>\[(?P<array_length>[0-9]*)?\])?$')
    type_match = type_regexp.match(msg_type)
    if type_match:
        return type_match.groupdict()
    else:
        return None


def ros_msg_loader(type_dict):
    module = importlib.import_module(type_dict['module'])
    msg_class = module.__dict__[type_dict['type']]
    if not msg_class:
        raise ImportError('Unable to find defined ROS Message type: {}'.format(type_dict['type']))
    else:
        return msg_class


def ros_msg_loader_str(msg_type):
    type_dict = ros_type_to_dict(msg_type)
    if type_dict:
        ros_msg_loader(type_dict)
    else:
        raise ImportError('Unable to find defined ROS Message type: {}'.format(msg_type))


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


def map_ros_types(ros_class):
    strategy_dict = {}
    slot_names = ros_class.__slots__
    slot_types = ros_class._slot_types
    slots_full = list(zip(slot_names, slot_types))
    for s_name, s_type in slots_full:
        type_dict = ros_type_to_dict(s_type)
        if type_dict:
            if not type_dict['complex']:
                if type_dict['array']:
                    parse_basic_arrays(s_name, type_dict, strategy_dict)
                elif type_dict['type'] is 'string':
                    strategy_dict[s_name] = st.text()
                elif type_dict['type'] is 'time':
                    strategy_dict[s_name] = time()
                elif type_dict['type'] is 'duration':
                    strategy_dict[s_name] = duration()
                else:  # numpy compatible ROS built-in types
                    strategy_dict[s_name] = npst.from_dtype(np.dtype(type_dict['type']))
            else:
                parse_complex_types(s_name, type_dict, strategy_dict)
    return dynamic_strategy_generator_ros(ros_class, strategy_dict)


def parse_basic_arrays(s_name, type_dict, strategy_dict):
    if type_dict['array_size']:
        array_size = int(type_dict['array_size'])
    else:
        array_size = None
    if type_dict['type'] == 'string':
        strategy_dict[s_name] = array(elements=string(), min_size=array_size, max_size=array_size)
    else:
        strategy_dict[s_name] = array(elements=npst.from_dtype(np.dtype(type_dict['type'])), min_size=array_size,
                                      max_size=array_size)


def parse_complex_types(s_name, type_dict, strategy_dict):
    if not type_dict['array']:
        strategy_dict[s_name] = map_ros_types(ros_msg_loader(type_dict))
    else:
        if type_dict['array_size']:
            strategy_dict[s_name] = array(elements=map_ros_types(ros_msg_loader(type_dict)),
                                          min_size=int(type_dict['array_size']),
                                          max_size=int(type_dict['array_size']))
        else:
            strategy_dict[s_name] = array(elements=map_ros_types(ros_msg_loader(type_dict)))


# A better approach. It returns an instance of a ROS msg directly, so no need for mapping! :)
@st.composite
def dynamic_strategy_generator_ros(draw, ros_class, strategy_dict):  # This generates existing ROS msgs objects
    aux_obj = ros_class()
    for key, value in strategy_dict.iteritems():
        setattr(aux_obj, key, draw(value))
    return aux_obj
