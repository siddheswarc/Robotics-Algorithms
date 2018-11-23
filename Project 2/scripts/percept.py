#!/usr/bin/env python

# import dependencies
import rospy
import random
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

rospy.init_node('percepter', anonymous = True)


def callback(data):
    global laserfeed
    laserfeed = data


# Function to find Random Sample Consensus (RANSAC)
def ransac(points, iterations):
    point_list = []
    inlier_list = []
    max_inlier = 0

    for i in range(iterations):

        # Randomly choosing two points to make a line
        p1 = random.choice(points)
        p2 = random.choice(points)

        # Find (x,y) co-ordinates of the start and end points of the line
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]

        marker.points = []
        first_point = Point()
        second_point = Point()

        num_of_inliers = 0
        threshold = 0.25

        for point in points:
            x0, y0 = point[0], point[1]

            try:
                # Find distance of point from line
                dist = abs(((y2-y1)*x0) - ((x2-x1)*y0) + ((x2*y1) - (y2*x1))) / math.sqrt(((y2-y1)**2) + ((x2-x1)**2))

            except ZeroDivisionError:
                continue

            if (dist <= threshold):
                num_of_inliers += 1

        # point_list stores start and end points of all lines found in ransac
        point_list.append((p1, p2))

        # inlier_list(i) stores the number of inliers of the line point_list(i)
        inlier_list.append(num_of_inliers)

        rate.sleep()


    # Find the line with maximum number of inliers
    for i in range(len(point_list)):
        if inlier_list[i] > max_inlier:
            first_point.x = point_list[i][0][0]
            first_point.y = point_list[i][0][1]
            second_point.x = point_list[i][1][0]
            second_point.y = point_list[i][1][1]
            max_inlier = inlier_list[i]

    marker.points.append(first_point)
    marker.points.append(second_point)


def percept():
    pub1 = rospy.Publisher("/visualization_marker1", Marker, queue_size = 10)
    pub2 = rospy.Publisher("/visualization_marker2", LaserScan, queue_size = 10)

    global rate
    rate = rospy.Rate(10)

    global marker
    marker = Marker()

    marker.header.frame_id = "/base_link"
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(1)

    # Set Marker scale
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.0

    # Set Marker color
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    # Number of iterations for RANSAC
    iterations = 10


    while not rospy.is_shutdown():
        rospy.Subscriber("/base_scan", LaserScan, callback)
        rate.sleep()

        points = []
        angle = laserfeed.angle_min

        for r in laserfeed.ranges:
            if r < 3:
                # Calculate the x and y coordinates of the points
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                coord = (x, y)

                # 'points' stores the x and y coordinates of all the points
                points.append(coord)

            angle += laserfeed.angle_increment

        if len(points) > 0:
            # Calling the RANSAC function
            ransac(points, iterations)

        pub2.publish(laserfeed)
        #rate.sleep()
        pub1.publish(marker)


if __name__ == '__main__':
    try:
        percept()
    except rospy.ROSInterruptException:
        pass
