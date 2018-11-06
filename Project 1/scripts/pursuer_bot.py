#!/usr/bin/env python

# import dependencies
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import math

velocity = Twist()
odom0 = Odometry()
odom1 = Odometry()

def callback0(data):
    pass

def callback1(data1):
    global odom1
    odom1 = data1

def pursuer_bot_pose():
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback1)
    br = tf.TransformBroadcaster()
    br.sendTransform((odom1.pose.pose.position.x, odom1.pose.pose.position.y, odom1.pose.pose.position.z),
                        tf.transformations.quaternion_from_euler(odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z),
                        rospy.Time.now(),
                        "/robot_1/odom",
                        "world")

# Controller function for the pursuer_bot
def pursuer_bot():
    listener = tf.TransformListener()
    publisher = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size = 10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback0)
        pursuer_bot_pose()

        try:
#	    now = rospy.Time.now()
#	    past = now - rospy.Duration(2)
#	    listener.waitForTransformFull('/robot_1/base_link', now, '/robot_0/base_link', past, "/world", rospy.Duration(1))
#            (trans, rot) = listener.lookupTransformFull('/robot_1/base_link', now, '/robot_0/base_link', past, "/world")

	    (trans, rot) = listener.lookupTransform('/robot_1/base_link', '/robot_0/base_link', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        velocity.linear.x = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        velocity.angular.z = 3 * math.atan2(trans[1], trans[0])

        publisher.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('pursuer_bot', anonymous = True)
        pursuer_bot()

    except rospy.ROSInterruptException:
        pass
