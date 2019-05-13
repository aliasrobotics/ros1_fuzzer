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

