"""
ROS Fuzzer base module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""

import importlib
import re
import numpy as np
import hypothesis.extra.numpy as npst
import hypothesis.strategies as st
from ros_basic_strategies import array, string, time, duration

try:
    import rospy
    import rosmsg
    import rospkg
except ImportError:
    print "Please install ROS first"


def ros_type_to_dict(msg_type):
    """
    Create a dictionary which values say if the ROS message type is complex (not basic), which is its parent
    ROS message module, its type, if it is an array and if so, its size.

    :param msg_type: ROS message type.
    :return: A dictionary which values say if the ROS message type is complex (not basic), which is its parent
             ROS message module, its type, if it is an array and if so, its size.
    """
    type_regexp = re.compile(
        r'^(?P<complex>(?P<module>[\w]+)/)?(?P<type>[\w]+)(?P<array>\[(?P<array_size>[0-9]*)?\])?$')
    type_match = type_regexp.match(msg_type)
    if type_match:
        return type_match.groupdict()
    else:
        return None


def ros_msg_loader(type_dict):
    """
    Dynamically import ROS message modules.

    :param type_dict: A dictionary which values say if the ROS message type is complex (not basic), which is its parent
                      ROS message module, its type, if it is an array and if so, its size.
    :return: The ROS message class. If the provided type does not exist, raises an import error.
    """
    try:
        module = importlib.import_module(type_dict['module'] + '.msg')
        return module.__dict__[type_dict['type']]
    except KeyError:
        raise KeyError('ROS message type: {} not included in message module: {}'.format(type_dict['type'],
                                                                                    type_dict['module']))
    except ImportError:
        raise ImportError('ROS message module: {} does not exist.'.format(type_dict['module']))
    except TypeError:
        raise TypeError('ROS message type: {} does not exist'.format(type_dict['type']))


def ros_msg_loader_str(msg_type):
    """
    Wrapper for the :func:`ros_msg_loader` to treat string type command line arguments.

    :param msg_type: A string type ROS message type (e.g. "Log").
    :return: The :func:`ros_msg_loader` function.
    """
    type_dict = ros_type_to_dict(msg_type)
    if type_dict:
        return ros_msg_loader(type_dict)
    else:
        raise ImportError('Unable to find defined ROS Message type: {}'.format(msg_type))


def create_publisher(topic, msg_type):
    """
    Create an instance of a ROS publisher object an initialize it.

    :param topic: The ROS topic that the publisher object is going to write in.
    :param msg_type: ROS message type (e.g. Log).
    :return: The newly created publisher object.
    """
    pub = rospy.Publisher(topic, msg_type, queue_size=10)
    rospy.init_node('fuzzer_node', anonymous=False)
    return pub


def map_ros_types(ros_class):
    """
    A recursive function that maps ROS message fields to Hypothesis strategies.

    :param ros_class: The ROS class to be fuzzed.
    :return: A function that generates Hypothesis strategies for a given ROS message type.
    """
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
    """
    Generate Hypothesis strategies for array types.

    :param s_name: Slot name to be parsed.
    :param type_dict: A dictionary which values say if the ROS message type is complex (not basic), which is its parent
                      ROS message module, its type, if it is an array and if so, its size.
    :param strategy_dict: A pointer to a dictionary to be filled with Hypothesis strategies.
    """
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
    """
    Generate Hypothesis strategies for complex ROS types.

    :param s_name: Slot name to be parsed.
    :param type_dict: A dictionary which values say if the ROS message type is complex (not basic), which is its parent
                      ROS message module, its type, if it is an array and if so, its size.
    :param strategy_dict: A pointer to a dictionary to be filled with Hypothesis strategies.
    """
    if not type_dict['array']:
        strategy_dict[s_name] = map_ros_types(ros_msg_loader(type_dict))
    else:
        if type_dict['array_size']:
            strategy_dict[s_name] = array(elements=map_ros_types(ros_msg_loader(type_dict)),
                                          min_size=int(type_dict['array_size']),
                                          max_size=int(type_dict['array_size']))
        else:
            strategy_dict[s_name] = array(elements=map_ros_types(ros_msg_loader(type_dict)))


@st.composite
def dynamic_strategy_generator_ros(draw, ros_class, strategy_dict):  # This generates existing ROS msgs objects
    """
    Generates Hypothesis strategies for a certain ROS class.

    :param ros_class: The ROS class to be filed with fuzzed values.
    :param strategy_dict: Strategy dictionary.
    :return: A pointer to a dictionary filled with Hypothesis strategies.
    """
    aux_obj = ros_class()
    for key, value in strategy_dict.iteritems():
        setattr(aux_obj, key, draw(value))
    return aux_obj
