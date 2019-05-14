import unittest
import rospy
from hypothesis import given
from std_msgs.msg import Header
from rosgraph_msgs.msg import Log
from ros_commons import map_ros_types
from hypothesis.strategies import text


class TestRosLogMessages(unittest.TestCase):

    def setUp(self):
        self.pub = rospy.Publisher('/rosout', Log, queue_size=10)
        self.node = rospy.init_node('fuzzer_node', anonymous=False)

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

