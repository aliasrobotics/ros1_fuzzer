"""
ROS Fuzzer CLI interface main module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
import logging
import rospy
from argparse import ArgumentParser
from hypothesis import given, settings, Verbosity
from ros_commons import ros_msg_loader_str, create_publisher, map_ros_types


def test_main_wrapper(msg_type, topic):
    """
    Wrapper for the main function. Publishes the fuzzed values for a given ROS message type.

    :param msg_type: ROS message type
    :param topic: ROS topic to be publish the fuzzed messages
    """
    pub = create_publisher(topic, msg_type)

    @settings(verbosity=Verbosity.verbose)
    @given(msg=map_ros_types(msg_type))
    def test_main(pub, msg):
        try:
            pub.publish(msg)
            print "publishing"
        except Exception as e:
            print "Error when publishing: {}".format(str(e))

    test_main(pub)
    rospy.signal_shutdown('shutdown')


def main():
    """
    Main method. Takes two command line arguments, parses them and calls :func:`test_main_wrapper`
    """
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    parser = ArgumentParser(description='ROS subscriber fuzzer')
    parser.add_argument('-m', '--message', help='Message type to be fuzzed.', required=True)
    parser.add_argument('-t', '--topic', help='Topic name to be fuzzed.', required=True)
    args = parser.parse_args()
    try:
        msg_type = ros_msg_loader_str(args.message)
        test_main_wrapper(msg_type, args.topic)
    except Exception as e:
        logger.critical('Exception occurred during execution --> ' + str(e))


if __name__ == '__main__':
    main()
