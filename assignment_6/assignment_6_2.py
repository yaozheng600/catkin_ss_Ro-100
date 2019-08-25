#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import random

class LaneProcessing: 

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("threshold", Image, queue_size = 10)
        self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.on_image, queue_size = 10) 
	self.cv_image = []
        rospy.init_node("lane_processing")
        
        self.rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		self.rate.sleep()

    def on_image(self, msg):
        try:		
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)
	
	cv2.waitKey(3)		
	crop_image = self.cv_image[180:350, ]
	#cv2.imshow('show crop image', crop_image)
	cv2.waitKey(3)	
        ret, bi_image = cv2.threshold(crop_image, 155, 255, cv2.THRESH_BINARY)
	#cv2.imshow('show bi image', bi_image)

	x_points, y_points = self.collect_points()
	N = len(x_points)
	k = 1000
	s = 2
	t = 0.5
	d = 0.2 * N
	m, b, index = self.find_model(x_points, y_points, N, k, s, t, d)
	x_1 = x_points[index[2]]
	y_1 = m * x_1 + b
	x_2 = x_points[index[len(index) -1]]
	y_2 = m * x_2 + b
	 
	cv2.line(crop_image, (int(x_1), int(y_1)), (int(x_2), int(y_2)), (0, 0, 255))
	print(len(index))
	print(str(x_1)+ "------------" + str(y_1))
	print(str(x_2) + "------------"+str( y_2))
	cv2.waitKey(3)
	cv2.imshow("first line", crop_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "mono8"))


    def collect_points(self):
	x_points = []
	y_points = []
	for y in range(self.cv_image.shape[0]):
	    for x in range(self.cv_image.shape[1]):
		if self.cv_image[y][x] >= 155:
		    x_points.append(x)
		    y_points.append(y)
	return x_points, y_points


    def find_model(self, X, Y, N, k, s, t, d):
	inliers_x = []
	inliers_y = []
	total_m = 0 
	square_x = 0 
	total_b_up = 0 
	total_b_down = 0 
	count = 0 

	for _ in range(k):
	    index = [] 
	    count_in = 0

	    r_1 = random.randint(1, N)
	    r_2 = random.randint(1, N)
	    x_1 = X[r_1]
	    y_1 = Y[r_1]
	    x_2 = X[r_2]
	    y_2 = Y[r_2]
	    m = (y_2 - y_1)/(x_2 - x_1 + 0.0000001) 
	    b = y_1 - m * x_1

	    for i in range(N):
	        y = m * X[i] + b
	        if (abs(y - Y[i]) < t):
		    count_in += 1
		    index.append(i)

	    if len(index) >= d or count_in > count:
	        inliers_x = []
	        inliers_y = []
	        for i in range(len(index)):
		    inliers_x.append(X[index[i]])
		    inliers_y.append(Y[index[i]])
	        mean_x = np.mean(inliers_x)
	        mean_y = np.mean(inliers_y)
	        for i in range(len(index)):
		    total_m += inliers_x[i] * inliers_y[i]
		    square_x += np.square(inliers_x[i])
		    total_b_up += (inliers_x[i] - mean_x) * (inliers_y[i] - mean_y)  
		    total_b_down += np.square(inliers_x[i] - mean_x)
	   
	        m_ = (total_m - len(index) *  mean_y) / (square_x - len(index) * np.square(mean_x))
	        b_ = total_b_up / total_b_down
	        index_list = index
	    
	    
	return m_, b_, index_list
	

if __name__ == "__main__":
    LaneProcessing()
