#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import scipy # use numpy if scipy unavailable
import scipy.linalg # use numpy if scipy unavailable



class lane:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("threshold", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.on_image, queue_size=1)
        self.cv_image = []  # 488*640

        rospy.init_node("lane",anonymous =True)

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_image(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8") #grayscale
        except CvBridgeError as e:
            print(e)
 
        cv2.waitKey(3)		
        crop_image = self.cv_image[180:350, ]
        points =self.collect_points()
        _,indexs =self.test()
        print()
        cv2.line(crop_image, (points[indexs[5],0],points[indexs[0],1]),(points[indexs[2000],0],points[indexs[10],1]), (0, 0, 255))
        cv2.imshow('show crop image', crop_image)
        cv2.waitKey(3)	
        ret, self.cv_image = cv2.threshold(crop_image, 155, 255, cv2.THRESH_BINARY)
        cv2.imshow('show bi image', self.cv_image)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "mono8"))
        
    def collect_points(self):
        points = []
        for y in range(self.cv_image.shape[0]):
           for x in range(self.cv_image.shape[1]):
               if self.cv_image[y,x] >= 155:
                   points.append([x,y])
        points = np.array(points)
        return points
 
    def random_partition(self,n, n_data):
        """return n random rows of data and the other len(data)-n rows"""
        all_idxs = np.arange(n_data) 
        np.random.shuffle(all_idxs)
        idxs1 = all_idxs[:n]
        idxs2 = all_idxs[n:]
        return idxs1, idxs2
    
    def ransac(self,data,model,n,k,t,d):
        iterations = 0
        bestfit = None
        besterr = np.inf 
        best_inlier_idxs = None
        
        while iterations < k:
            maybe_idxs, test_idxs =self.random_partition(n,data.shape[0])
            maybeinliers = data[maybe_idxs,:]
            test_points = data[test_idxs]
            maybemodel = model.fit(maybeinliers)
            test_err = model.get_error( test_points, maybemodel)
            also_idxs = test_idxs[test_err < t]
            alsoinliers = data[also_idxs,:]
            
            if len(alsoinliers) > d:
                betterdata = np.concatenate((maybeinliers,alsoinliers))
                bettermodel = model.fit(betterdata)
                better_errs = model.get_error(betterdata,bettermodel)
                thiserr = np.mean(better_errs)
                if thiserr < besterr:
                    bestfit = bettermodel
                    besterr = thiserr
                    best_inlier_idxs = np.concatenate((maybe_idxs,also_idxs))
            iterations+=1
        
        if bestfit is None:
            raise ValueError("did not meet fit acceptance criteria")
        else:
            return bestfit,best_inlier_idxs
    
    def test(self):
        all_data = self.collect_points()
        model = LinearLeastSquaresModel()
        
        ransac_fit, ransac_data = self.ransac(all_data,model,50, 1000, 10, 2)
        print(ransac_fit)
        print(ransac_data)
        return ransac_fit, ransac_data
class LinearLeastSquaresModel:
    
    def __init__(self,debug=False):
        self.debug = debug
    def fit(self, data):
        A = np.hsplit(data,2)
        X = np.hstack((A[0],np.ones((data.shape[0],1))))
        Y = A[1]
        m,b = np.linalg.lstsq(X,Y)[0]
        return m,b
    def get_error( self, data, model):
        A = np.vstack([data[:,i] for i in self.input_columns]).T
        B = np.vstack([data[:,i] for i in self.output_columns]).T
        B_fit = scipy.dot(A,model)
        err_per_point = np.sum((B-B_fit)**2,axis=1) # sum squared error per row
        return err_per_point
            
   

if __name__ == "__main__":
    l = lane()
):
        A = np.vstack([data[:,i] for i in self.input_columns]).T
        B = np.vstack([data[:,i] for i in self.output_columns]).T
        B_fit = scipy.dot(A,model)
        err_per_point = np.sum((B-B_fit)**2,axis=1) # sum squared error per row
        return err_per_point
            
   

if __name__ == "__main__":
    l = lane()
