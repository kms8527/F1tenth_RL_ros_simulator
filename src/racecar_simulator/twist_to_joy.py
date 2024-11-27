#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    # Convert Twist to Joy here
    joy_msg = Joy()

    # Assuming that your joystick control uses axes[0] for linear and axes[1] for angular
    joy_msg.axes.append(data.linear.x)
    joy_msg.axes.append(data.angular.z)

    pub.publish(joy_msg)

rospy.init_node('twist_to_joy')
rospy.Subscriber("cmd_vel", Twist, callback)
pub = rospy.Publisher('joy', Joy, queue_size=1)

rospy.spin()
