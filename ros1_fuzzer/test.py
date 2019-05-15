"""
ROS Fuzzer example test cases module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
import unittest
import rospy
from hypothesis import given, settings, Verbosity
from numpy import float64
from std_msgs.msg import Header
from rosgraph_msgs.msg import Log

from ros1_fuzzer.process_handling import FuzzedLocalProcessHandler
from ros1_fuzzer.ros_basic_strategies import array
from ros_commons import map_ros_types
from hypothesis.strategies import text
from sensor_msgs.msg import JointState

class TestRosLogMessages(unittest.TestCase):

    def setUp(self):
        self.pub = rospy.Publisher('/rosout', Log, queue_size=10)
        self.node = rospy.init_node('fuzzer_node', anonymous=False)
        self.process_handler = FuzzedLocalProcessHandler('/example_node')

    def tearDown(self):
        rospy.signal_shutdown('Shutting down fuzzer node')

    @given(header=map_ros_types(Header))
    def test_fuzz_log_message(self, header):
        log = Log()
        log.header = header
        self.pub.publish(log)

    @given(log=map_ros_types(Log))
    def test_fuzz_log_message_exclude(self, log):
        log.name = 'Fixed name'
        self.pub.publish(log)

    @given(log=map_ros_types(Log), name=text(min_size=10, max_size=20))
    def test_fuzz_log_message_arbitrary(self, log, name):
        log.name = name
        self.pub.publish(log)

    @settings(max_examples=5000, verbosity=Verbosity.verbose)
    @given(array(elements=float64(), min_size=6, max_size=6))
    def test_fuzz_message_jointstate_effort(self, fuzzed_fields):
        joint_state_message = JointState()
        joint_state_message.effort = fuzzed_fields
        self.pub.publish(joint_state_message)
        assert self.process_handler.check_if_alive() is True

    @settings(max_examples=5000, verbosity=Verbosity.verbose)
    @given(array(elements=float64(), min_size=6, max_size=6),
           array(elements=float64(), min_size=6, max_size=6),
           array(elements=float64(), min_size=6, max_size=6))
    def test_fuzz_message_jointstate_all(self, positions, velocities, efforts):
        joint_state_message = JointState()
        joint_state_message.position = positions
        joint_state_message.velocity = velocities
        joint_state_message.effort = efforts
        self.pub.publish(joint_state_message)
        assert self.process_handler.check_if_alive() is True