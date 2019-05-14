Fuzzer usage
============

The fuzzer is able to work in a standalone manner, by calling it to fuzz a full message structure via CLI,
or by designing custom tests tha fuzz or exclude different fields of the messages.


CLI Usage
---------

The fuzzer can be directly invoked from command line. Ensure that the ROS workspace is sourced before proceeding.
Message types follow the ROS naming scheme.

.. code-block:: bash

    $ source /opt/ros/melodic/setup.bash
    $ ros_fuzzer -t <topic> -m <message_type>


Here is a usage example for performing a Log message fuzzing over the /rosout topic:

.. code-block:: bash

    $ source /opt/ros/melodic/setup.bash
    $ ros_fuzzer -t /rosout -m rosgraph_msgs/Log


Usage as Unit Testing counterpart
---------------------------------

Test cases can use the provided Hypothesis strategy generation functions to get fuzzed messages that can be modified and
used for different purposes. Fuzzed test cases follow the same mechanisms as standard data test cases,
obtaining tbe fuzzed message as a parameter to the test case.
The following example shows a simple test case that makes use of a fuzzed Log message,
that is then modified before being sent.

.. code-block:: python
    :caption: Example unittest test case

    @given(log=map_ros_types(Log))
    def test_fuzz_log_message_exclude(self, log):
        log.name = 'Fixed name'
        self.pub.publish(log)


The :func:`ros1_fuzzer.ros_commons.map_ros_types` function provides a dynamic strategy for the defined ROS Message class,
that correctly sets up each of the elements of the message with corresponding data type fuzzers.
Examples can be extended to even fuzz different message types or subelements independently.
Built in hypothesis :mod:`hypothesis.strategies` can be used as well.
The :func:`hypothesis.given` decorator runs the decorated function with all the defined fuzz cases.

.. code-block:: python
    :caption: Example unittest with multiple parameters.

    @given(log=map_ros_types(Log), header=map_ros_types(Header), name=st.text(min_length=1, max_length=20))
    def test_fuzz_log_message_parameters(log, header, name):
        log.name = name
        log.header = header
        self.pub.publish(log)


