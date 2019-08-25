#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate
import os
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped

class SplineProcessing:

    def __init__(self):
	rospy.init_node("spline_publisher")
        self.marker_1_pub = rospy.Publisher("visualization_msgs/Marker1", Marker, queue_size = 10)
	self.marker_2_pub = rospy.Publisher("visualization_msgs/Marker2", Marker, queue_size = 10)
	#self.marker_sub = rospy.Subscriber("/clicked_point", PointStamped, self.get_point, queue_size = 10)
	self.sphere_pub = rospy.Publisher("visualization_msgs/Sphere", Marker, queue_size = 10)
	self.marker_sub = rospy.Subscriber("/clicked_point", PointStamped, self.get_lookahead, queue_size = 10)
	self.lookahead_1_pub = rospy.Publisher("visualization_msgs/Lookahead1", Marker, queue_size = 10)
	self.lookahead_2_pub = rospy.Publisher("visualization_msgs/Lookahead2", Marker, queue_size = 10)
        self.rate = rospy.Rate(10)
		
	while not rospy.is_shutdown():
	    path_1 = "/home/peng/catkin_ss_Ro-100/src/assignment_8/src/lane1.npy"
	    lane_1 = np.load(path_1)
	    path_2 = "/home/peng/catkin_ss_Ro-100/src/assignment_8/src/lane2.npy"
	    lane_2 = np.load(path_2) 
	    lane_p_1 = lane_1[[0, 100, 150, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1276], :]
	    lane_p_2 = lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :]
	    self.spline_1_1 = interpolate.CubicSpline(lane_p_1[:,0], lane_p_1[:,1])
	    self.spline_1_2 = interpolate.CubicSpline(lane_p_1[:,0], lane_p_1[:,2])
	    self.spline_2_1 = interpolate.CubicSpline(lane_p_2[:,0], lane_p_2[:,1])
	    self.spline_2_2 = interpolate.CubicSpline(lane_p_2[:,0], lane_p_2[:,2])
	    self.samples = np.arange(0.01, 12.77, 0.01)
	    self.samples_2 = np.arange(0.01, 14.77, 0.01)
	
	    marker = Marker()
	    marker.type = 4
	    marker.header.frame_id = "/map"
	    marker.text= "line_strip"
	    marker.color.a = 1
	    marker.color.r = 1
	    marker.color.g = 1
	    marker.color.b = 0
	    marker.pose.position.x = 0
	    marker.pose.position.y = 0
	    marker.pose.position.z = 0
	    marker.scale.x = 0.01
	    marker.scale.y = 0.01
	    marker.scale.z = 0.01
	
	    for i in range(len(self.samples)):
	        marker.points.append(Point(self.spline_1_1(self.samples[i]), self.spline_1_2(self.samples[i]), 0))
            self.marker_1_pub.publish(marker)

	    marker = Marker()
	    marker.type = 4
	    marker.header.frame_id = "/map"
	    marker.text= "line_strip"
	    marker.color.a = 1
	    marker.color.r = 1
	    marker.color.g = 1
	    marker.color.b = 0
	    marker.pose.position.x = 0
	    marker.pose.position.y = 0
	    marker.pose.position.z = 0
	    marker.scale.x = 0.01
	    marker.scale.y = 0.01
	    marker.scale.z = 0.01
	
	    for i in range(len(self.samples_2)):
	        marker.points.append(Point(self.spline_2_1(self.samples_2[i]), self.spline_2_2(self.samples_2[i]), 0))
            self.marker_2_pub.publish(marker)
	    self.rate.sleep()

    def get_lookahead(self):
	print("")

    def distance(self, x_1, y_1, x_2, y_2):
        return np.sqrt(np.square(x_1 - x_2) + np.square(y_1 - y_2))

    def closest_cal(self, start, end, x, y):
        middle = (start + end)//2
        start_point = self.distance(self.X[start], self.Y[start], x, y)
	end_point = self.distance(self.X[end], self.Y[end], x, y)
        if middle == self.last_middle:
            return 
	self.last_middle = middle
        if start_point < end_point:
            self.closest_cal(start, middle, x, y)
        else:
            self.closest_cal(middle, end, x, y)

    def closest_point(self, x, y):
        self.X = self.spline_1_1(self.samples)
        self.Y = self.spline_1_2(self.samples)
        n = len(self.samples)
        middle = n//2
	self.last_middle = middle
        if n > 1:
            start_point = self.distance(self.X[0], self.Y[0], x, y)
	    middle_point = self.distance(self.X[middle], self.Y[middle], x, y)
	    if start_point <= middle_point:
		start = 0
	    else:
		start = middle
	    middle = middle//2
	    start_middle_point = self.distance(self.X[middle], self.Y[middle], x, y)
	    end_middle_point = self.distance(self.X[int(1.5 * middle)], self.Y[int(1.5 * middle)], x, y)
	    if start_middle_point <= end_middle_point:
		end = middle
	    else:
		end = int(1.5 * middle)
	    if start > end:
		swp = start
		start = end
		end = swp
            self.closest_cal(start, end, x, y)

    def get_point(self, msg):
	marker = Marker()
	marker.type = 2
	marker.header.frame_id = "/map"
	marker.color.a = 1
	marker.color.r = 1
	marker.color.g = 0
	marker.color.b = 0
	marker.scale.x = 0.1
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	self.closest_point(msg.point.x, msg.point.y)
	marker.pose.position.x = self.X[self.last_middle]
	marker.pose.position.y = self.Y[self.last_middle]
	marker.points.append(Point(self.X[self.last_middle], self.Y[self.last_middle], 0))
	print(marker)
	self.sphere_pub.publish(marker)

    def get_lookahead(self, msg):
	marker1 = Marker()
	marker1.type = 2
	marker1.header.frame_id = "/map"
	marker1.color.a = 1
	marker1.color.r = 1
	marker1.color.g = 0
	marker1.color.b = 0
	marker1.scale.x = 0.1
	marker1.scale.y = 0.1
	marker1.scale.z = 0.1

	marker2 = Marker()
	marker2.type = 2
	marker2.header.frame_id = "/map"
	marker2.color.a = 1
	marker2.color.r = 0
	marker2.color.g = 0
	marker2.color.b = 1
	marker2.scale.x = 0.1
	marker2.scale.y = 0.1
	marker2.scale.z = 0.1
	x = round(msg.point.x, 2)
	y = round(msg.point.y, 2)
	self.X = [round(elem, 2) for elem in self.spline_1_1(self.samples)]
        self.Y = [round(elem, 2) for elem in self.spline_1_2(self.samples)]
	self.X_2 = [round(elem, 2) for elem in self.spline_2_1(self.samples_2)]
        self.Y_2 = [round(elem, 2) for elem in self.spline_2_2(self.samples_2)]
	l = [i for i in range(len(self.X_2)) if self.X_2[i] == x]
	print(l)
	if x in self.X:
	    marker1.pose.position.x = self.X[l[0]]
	    marker1.pose.position.y = msg.point.y
	    marker1.points.append(Point(self.X[l[0]], msg.point.y,  0))
	    marker2.pose.position.x = self.X[l[1]]
	    marker2.pose.position.y = msg.point.y
	    marker2.points.append(Point(self.X[l[1]], msg.point.y,  0))
	    	
	else:
	    marker1.pose.position.x = msg.point.x
	    marker1.pose.position.y = self.Y[l[0]]
	    marker1.points.append(Point(msg.point.x, self.Y[l[0]], 0))
	    marker2.pose.position.x = msg.point.x
	    marker2.pose.position.y = self.Y[l[1]]	
	    marker2.points.append(Point(msg.point.x, self.Y[l[1]], 0))
		    
	#print(marker1)
	self.lookahead_1_pub.publish(marker1)
	self.lookahead_2_pub.publish(marker2)
        

if __name__ == "__main__":
	SplineProcessing()

