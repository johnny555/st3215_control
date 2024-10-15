# write a python node to publish a joint_states message on a topic

import  rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState



rclpy.init()

node = Node("pubber")

pub = node.create_publisher(JointState, '/desired_joint_state', 10)

jst = JointState()
# set the joint names and positions in the message
jst.name = ['joint1']
jst.position = [000.0]

# publish the message on the topic
pub.publish(jst)







