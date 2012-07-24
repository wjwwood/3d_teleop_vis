#!/usr/bin/env python

"""
This is a ROS node that takes a topic in, delays a given
amount of time, and then republishes it, unchanged.
"""

import sys
from time import sleep
from Queue import Queue, Empty

import roslib; roslib.load_manifest('teleop_client')
import rospy
from rostopic import get_topic_type

from std_msgs.msg import Float32

class LatencyInducerNode(object):
    """Latency Inducer Node"""
    def __init__(self):
        rospy.init_node('latency_inducer')

        self.in_topic = None
        self.out_topic = None
        self.delay_period = None
        self.queue = Queue()
        self.pub = None

        self.get_ros_params()

        self.setup_ros_comms()

        while not rospy.is_shutdown():
            try:
                msg, time = self.queue.get(True, 0.1)
                now = rospy.Time.now().to_sec()
                diff = time - now
                if diff > 0.001:
                    sleep(diff)
                if self.pub:
                    self.pub.publish(msg)
            except Empty as e:
                pass

    def on_latency_source(self, msg):
        self.delay_period = msg.data

    def on_in_topic(self, msg):
        time = rospy.Time.now().to_sec()
        try:
            time = msg.header.stamp.to_sec()
        except Exception as e:
            pass
        if self.delay_period == None:
            return
        time += self.delay_period
        if self.pub == None:
            pkg, msg_name = msg._connection_header['type'].split('/')
            roslib.load_manifest(pkg)
            import_cmd = 'from {0}.msg import {1}'.format(pkg, msg_name)
            exec import_cmd
            create_pub_cmd = 'self.pub = rospy.Publisher(self.out_topic, {0})'.format(msg_name)
            exec create_pub_cmd
        self.queue.put((msg, time))

    def setup_ros_comms(self):
        """Sets up ROS communications"""
        rospy.Subscriber('/latency_source', Float32, self.on_latency_source)
        rospy.Subscriber(self.in_topic, rospy.AnyMsg, self.on_in_topic)

    def get_ros_params(self):
        """Gets the ROS parameters"""
        self.in_topic = rospy.get_param('~in_topic')
        self.out_topic = rospy.get_param('~out_topic', self.in_topic + '_delayed')

if __name__ == '__main__':
    lin = LatencyInducerNode()
