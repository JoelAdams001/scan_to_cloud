#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import math as m
import tf2_ros
import geometry_msgs.msg

def callback(laser):
    print("callback")
    try:
        #Transform frame from world to lidar
        T = tfbuffer.lookup_transform('odom', 'front_laser', laser.header.stamp)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
    #Convert quaternion to rotation matrix
    qx = T.transform.rotation.x
    qy = T.transform.rotation.y
    qz = T.transform.rotation.z
    qw = T.transform.rotation.w
    x = T.transform.translation.x
    y = T.transform.translation.y
    z = T.transform.translation.z
    T_mat = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw, x], [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw, y], [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2, z], [0, 0, 0, 1]])
    inc = 0; #increment starts at 0 before loop
        print(p)
        print(p)
        print(p)
        print(p)
        print(p)
    cloud = np.empty((0,4), float)
    for range in laser.ranges:
        ang = laser.range_min + inc*laser.angle_increment #Find the angle for the specific point in message
        x2 = range*m.sin(ang)
        y2 = range*m.cos(ang)
        p = T_mat.dot(np.array([[x2, y2, 0, 1]]))
        cloud = np.append(cloud, p, axis=0)
        inc = inc + 1
    p[p >= 1E308] = 0
    p[p <= 1E308] = 0
    p = T_mat.dot(p)

rospy.init_node('scan_to_cloud', anonymous=True)
tfbuffer = tf2_ros.Buffer()
tflisten = tf2_ros.TransformListener(tfbuffer)
rospy.Subscriber("/front/scan", LaserScan, callback)
rate = rospy.Rate(10.0)
rospy.spin()
