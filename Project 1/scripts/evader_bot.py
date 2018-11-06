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
laserfeed = []
odom = Odometry()

def callback0(data0):
    global laserfeed

    # laserfeed is now an object of LaserScan
    laserfeed = data0.ranges[:]

def callback1(data1):
    global odom

    # odom is now an object of Odometry
    odom = data1

# Function to find minimum value in a list
def find_min(lst):

    # Setting minimum distance to the maximum value of our laser's range
    min_dist = 3

    for i in lst:
        if (i < min_dist):
	    min_dist = i
    return min_dist

# Function to set the bot's linear velocity to 0
def bot_stop():
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 0


# Function to set the bot's angular velocity to 0 and make it move linearly at a constant speed 'x'
def bot_move(x):

    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0

    velocity.linear.x = x
    velocity.linear.y = 0
    velocity.linear.z = 0

# Function to make the bot stop moving linearly and rotate it at a random angle between 0 and pi
def bot_rotate():

    bot_stop()

    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = random.randint(0, int(math.pi))

    bot_stop()

def evader_bot_pose():
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback1)
    br = tf.TransformBroadcaster()
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
                        tf.transformations.quaternion_from_euler(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
                        rospy.Time.now(),
                        "/odom",
                        "world")

# Controller function for the evader_bot
def evader_bot():
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.Subscriber("/base_scan", LaserScan, callback0)

        min_dist = find_min(laserfeed)

        if (min_dist < 0.85):
            bot_rotate()
            rate.sleep()

        else:
            bot_move(2)

        publisher.publish(velocity)

	evader_bot_pose()


if __name__ == '__main__':

    try:
        rospy.init_node('evader_bot', anonymous = True)
        evader_bot()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
