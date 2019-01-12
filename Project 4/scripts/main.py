#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: Siddheswar C
# @Email: innocentdevil.sid007@gmail.com


import rospy
import rosbag
import numpy as np
import math
from tf.transformations import euler_from_quaternion


def calculate_gaussian(x, mean, sd):
    x = float(x - mean)/sd
    return math.exp(-x*x/2.0) / math.sqrt(2.0*math.pi) / sd


def motion_model(rot1, trans, rot2, cell):
    new_cell = np.copy(cell)
    for x1 in range(cell.shape[0]):
        for y1 in range(cell.shape[1]):
            for theta1 in range(cell.shape[2]):
                if new_cell[x1][y1][theta1] < threshold:
                    continue

                sum = 0
                for x2 in range(cell.shape[0]):
                    for y2 in range(cell.shape[1]):
                        for theta2 in range(cell.shape[2]):
                            r1 = np.degrees(math.atan2((y2*20+10)-(y1*20+10),(x2*20+10)-(x1*20+10))) - my_pose[2] #- 180

                            # We add/subtract 360 as atan2 returns (-pi, pi)
                            if r1>180:
                                r1 -= 360
                            if r1<180:
                                r1+= 360
                            p1 = calculate_gaussian(r1, rot1, rotsd)

                            dist = math.sqrt((((x2*20+10)-(x1*20+10))**2) + (((y2*20+10)-(y1*20+10))**2))
                            p2 = calculate_gaussian(dist, trans, transsd)

                            r2 = rot2 - my_pose[2] - r1 #+180

                            # We add/subtract 360 as atan2 returns (-pi, pi)
                            if r2>180:
                                r2 -= 360
                            if r2<180:
                                r2 += 360
                            p3 = calculate_gaussian(r2, rot2, rotsd)

                            p = p1*p2*p3*new_cell[x1][y1][theta1]
                            cell[x2][y2][theta2] = p

    return cell


def observation_model(tagNum, ranges, bearings, cell):
    new_cell = np.copy(cell)
    sum = 0
    for x in range(cell.shape[0]):
        for y in range(cell.shape[1]):
            for theta in range(cell.shape[2]):
                r1 = np.degrees(math.atan2((landmarks.item(tagNum, 1) - (y*20+10)), (landmarks.item(tagNum, 0) - (x*20+10)))) - thetas[theta]

                # We add/subtract 360 as atan2 returns (-pi, pi)
                if r1>180:
                    r1 -= 360
                if r1<180:
                    r1+=360
                p1 = calculate_gaussian(r1, bearings, rotsd)

                dist = math.sqrt(((landmarks.item(tagNum, 0) - (x*20+10)) ** 2) + ((landmarks.item(tagNum, 1) - (y*20+10)) ** 2))
                p2 = calculate_gaussian(dist, ranges, transsd)

                p = p1 * p2

                prob = new_cell[x][y][theta] * p
                cell[x][y][theta] = prob
                sum += prob
    cell = cell/sum
    return cell


def read_rosbag(cell):
    bag = rosbag.Bag(r'../bag/grid.bag')
    file = open(r'../output/trajectory.txt', 'w')

    for topic, msg, t in bag.read_messages():
        if(topic == 'Movements'):

            rotation1 = msg.rotation1
            rotation2 = msg.rotation2

            rotation1 = np.degrees(euler_from_quaternion([rotation1.x, rotation1.y, rotation1.z, rotation1.w]))
            rotation2 = np.degrees(euler_from_quaternion([rotation2.x, rotation2.y, rotation2.z, rotation2.w]))

            translation = msg.translation * 100

            cell = motion_model(rotation1[2], translation, rotation2[2], cell)

        if(topic == 'Observations'):
            ranges = msg.range * 100
            bearings = msg.bearing
            tagNum = msg.tagNum
            bearings = np.degrees(euler_from_quaternion([bearings.x, bearings.y, bearings.z, bearings.w]))

            cell = observation_model(tagNum, ranges, bearings[2], cell)

            # Finding the indecx of the maximum value in the array
            index = np.argmax(cell)
            smallest = -1
            x,y,z = cell.shape
            yindex = index/z
            xindex = yindex/y
            xindex = xindex%x
            yindex = yindex%y
            zindex = index%z
            print(xindex+1, yindex+1, zindex)
            file.write(str(xindex+1) + ',' + str(yindex+1) + ',' + str(zindex) + '\n')

    bag.close()


if __name__ == '__main__':
    global transsd, rotsd, grid_size, cell_size, discretization_size, landmarks, threshold
    grid_size = 700
    cell_size = 20
    landmarks = np.asarray([[125, 525], [125, 325], [125, 125], [425, 125], [425, 325], [425, 525]])
    transsd = 10
    rotsd = 45
    discretization_size = 45
    threshold = 0.00000001

    global cell, new_cell, my_pose, thetas
    cell = np.zeros([grid_size/cell_size, grid_size/cell_size, 360/discretization_size])
    my_pose = [12, 28, int((200.52/discretization_size)+1)]
    cell[my_pose[0], my_pose[1], my_pose[2]] = 1
    thetas = np.zeros([360/discretization_size])
    tmp = -180

    for i in range(len(thetas)):
        thetas[i] = tmp
        tmp = tmp + discretization_size

    print(np.argwhere(cell.max()==cell))

    try:
        rospy.init_node('bayes_localization')
        read_rosbag(cell)

    except rospy.ROSInterruptException:
        pass
