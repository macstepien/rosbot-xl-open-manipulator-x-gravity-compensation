#!/usr/bin/env python

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion

rospy.init_node("cmd_vel_topic_publisher")
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)

initial_x = None
initial_theta = None

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

rate = rospy.Rate(5.0)

while not rospy.is_shutdown():
    try:
        end_effector_tf = tf_buffer.lookup_transform(
            "world", "end_effector_link", rospy.Time(0)
        )
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        rospy.logwarn("No tf")
        rate.sleep()
        continue

    x_pos = end_effector_tf.transform.translation.x
    quaternion = [
        end_effector_tf.transform.rotation.x,
        end_effector_tf.transform.rotation.y,
        end_effector_tf.transform.rotation.z,
        end_effector_tf.transform.rotation.w,
    ]

    euler_angles = euler_from_quaternion(quaternion)
    theta = euler_angles[2]

    if initial_x == None or initial_theta == None:
        initial_x = x_pos
        initial_theta = theta

    x_diff = -(x_pos - initial_x)

    theta_diff = math.fmod((theta - initial_theta), 2 * math.pi)
    if theta_diff > math.pi:
        theta_diff = theta_diff - 2 * math.pi
    elif theta_diff < -math.pi:
        theta_diff = theta_diff + 2 * math.pi

    if abs(x_diff) < 0.01:
        x_diff = 0.0

    if abs(theta_diff) < 0.1:
        theta_diff = 0.0

    x_diff *= 4

    cmd_vel_msg = Twist(Vector3(-x_diff, 0, 0), Vector3(0, 0, theta_diff))
    cmd_vel_pub.publish(cmd_vel_msg)

    rate.sleep()
