import rospy
from ros_commons import ros_msg_loader_str, create_publisher, map_ros_types
from hypothesis import given, settings, Verbosity
import logging
from argparse import ArgumentParser


def test_main_wrapper(msg_type, topic):
    pub = create_publisher(topic, msg_type)
    @settings(verbosity=Verbosity.verbose)
    @given(msg=map_ros_types(msg_type))
    def test_main(pub, msg):
        try:
            pub.publish(msg)
        except:
            pass

    test_main(pub)
    rospy.signal_shutdown('shutdown')


def main():
    """
    Main method
    """
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    parser = ArgumentParser(description='ROS subscriber fuzzer')
    parser.add_argument('-m', '--message', help='Message type to be fuzzed.', required=True)
    parser.add_argument('-t', '--topic', help='Topic name to be fuzzed.', required=True)
    args = parser.parse_args()
    if (args.message is None or args.topic is None) and args.length is None:
        parser.print_help()
    else:
        try:
            test_main_wrapper(ros_msg_loader_str(args.message), args.topic)
        except Exception as e:
            logger.critical('Exception occurred during execution --> ' + str(e))


if __name__ == '__main__':
    main()
    
