from ros_commons import *
from hypothesis import given, settings, Verbosity
import logging
from argparse import ArgumentParser
#import pydevd_pycharm

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
  #  pydevd_pycharm.settrace('172.20.10.15', port=6666, stdoutToServer=True, stderrToServer=True)
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
            if check_msg_type(ros_msg_list(), args.message):
                test_main_wrapper(eval(args.message), args.topic)
            else:
                logger.warning('Invalid ROS data type')
        except Exception as e:
            logger.critical('Exception occurred during execution --> ' + str(e))


if __name__ == '__main__':
    main()
    
