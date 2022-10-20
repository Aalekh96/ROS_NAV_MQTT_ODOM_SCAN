import msgpack
from unittest.mock import MagicMock
from paho.mqtt import client as mqtt_client
from random import randrange,uniform
import time

import rosgraph
import rospy
from nav_msgs.msg import Odometry  #to read the odometry
import paho.mqtt.client as mqtt
from std_msgs.msg import Bool, String, Float64

import logging
import sys
import time
import unittest
from logging import getLogger

logging.basicConfig(stream=sys.stderr)
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)



class TestMqttBridge(unittest.TestCase):



    def setup(self):
        self.mqtt_callback_odom=MagicMock()
        self.mqtt_callback_echo=MagicMock()
        self.mqttBroker="localhost"
        self.client=mqtt_client.Client("mqtt_pose")
        self.client.message_callback_add("odom", self.mqtt_callback_odom)
        self.client.connect(self.mqttBroker,1883)
        self.client.subscribe("odom")
        self.mqttc.subscribe("echo")
        self.client.loop_start()


        rospy.init_node("check_pub")
        self.ros_callback_odom=MagicMock()
        self.ros_callback_pong = MagicMock()
        self.subscriber_odom=rospy.Subscriber("/odom", Odometry,self.ros_callback_odom)
        self.subscriber_pong = rospy.Subscriber("/pong", Odometry, self.ros_callback_pong)
        #self.subscriber_echo = rospy.Subscriber("/echo", String, self.ros_callback_echo)



    def tearDown(self):
        self.subscriber_odom.unregister()
        self.subscriber_pong.unregister()
        #self.subscriber_echo.unregister()
        self.client.loop_stop()
        self.client.disconnect()
    

    def get_publisher(self,topic_path,msg_type,**kwargs):
        pub=rospy.Publisher(topic_path,msg_type,**kwargs)
        num_subs=len(self._get_subscribers(topic_path))
        for i in range (20):
            num_cons=pub.get_num_connections()
            if num_cons==num_subs:
                return pub
            time.sleep(0.1)
        self.fail("failed to get publisher")

    def _get_subscribers(self, topic_path):
        ros_master = rosgraph.Master('rostopic')
        topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
        state = ros_master.getSystemState()
        subs = []
        for sub in state[1]:
            if sub[0] == topic_path:
                subs.extend(sub[1])
        return subs

    def _wait_callback(self, callback_func):
        for i in range(10):
            if callback_func.called:
                return
            time.sleep(0.1)
        self.fail("callback doesn't be triggered")

    
    def test_ping_pong(self):
        publisher = self.get_publisher("/odom", Odometry, queue_size=10)
        publisher.publish(msg.pose.pose)
        self._wait_callback(self.ros_callback_odom)
        self.ros_callback_odom.assert_called_once_with(msg.pose.pose)

        self._wait_callback(self.ros_callback_pong)  
        self.ros_callback_pong.assert_called_once_with(msg.pose.pose)
        self.mqtt_callback_odom.assert_called_once()
        msg = self.mqtt_callback_odom.call_args[0][2]
        self.assertEqual(msg.topic, "odom")
        self.assertEqual(msg.payload, msgpack.dumps({"data": True}))

if __name__ == '__main__':
    import rostest
    rostest.rosrun('mqtt_bridge', 'mqtt_bridge_test', TestMqttBridge)
