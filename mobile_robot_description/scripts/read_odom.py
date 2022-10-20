#! /usr/bin/env python
import logging
import sys
import time
import unittest
from logging import getLogger
import unittest
from unittest.mock import MagicMock
import rospy
import paho.mqtt.client as mqtt
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,String,Float64
import rosgraph

logging.basicConfig(stream=sys.stderr)
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)


class mqttbridge(unittest.TestCase):



    def setUp(self):
        #mqtt
        self.mqtt_callback_odom=MagicMock()
        self.mqtt_callback_echo=MagicMock()
        self.mqttc.connect("localhost", 1883)
        self.mqttc.message_callback_add("odometry_sent", self.mqtt_callback_ping)
        self.mqttc.message_callback_add("echo", self.mqtt_callback_echo)
        self.mqttc.subscribe("odometry")
        self.mqttc.subscribe("echo")
        self.mqttc.loop_start()

        #ros
        rospy.init_node('mqtt_bridge_test_node')
        rospy.init_node("check_odom")
        self.ros_callback_ping = MagicMock()
        self.ros_callback_pong = MagicMock()
        self.ros_callback_echo = MagicMock()
        self.ros_callback_back = MagicMock()
        self.subscriber_ping = rospy.Subscriber("/ping", Bool, self.ros_callback_ping)
        self.subscriber_pong = rospy.Subscriber("/pong", Bool, self.ros_callback_pong)
        self.subscriber_echo = rospy.Subscriber("/echo", String, self.ros_callback_echo)
        self.subscriber_back = rospy.Subscriber("/back", String, self.ros_callback_back)


    def callback(msg):
         print (msg.pose.pose)

    def get_publisher(self, topic_path, msg_type, **kwargs):
        # wait until the number of connections would be same as ros master
        pub = rospy.Publisher(topic_path, msg_type, **kwargs)
        num_subs = len(self._get_subscribers(topic_path))
        for i in range(20):
            num_cons = pub.get_num_connections()
            if num_cons == num_subs:
                return pub
            time.sleep(0.1)
        self.fail("failed to get publisher")

    def _get_subscribers(self, topic_path):
        ros_master = rosgraph.Master('/rostopic')
        topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
        state = ros_master.getSystemState()
        subs = []
        for sub in state[1]:
            if sub[0] == topic_path:
                subs.extend(sub[1])
        return subs




if __name__=="__main__":
    rospy.init_node("check_odom")
    odom_sub=rospy.Subscriber("/odom", Odometry,callback)

    rospy.spin()

