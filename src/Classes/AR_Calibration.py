#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from transforms3d.euler import euler2mat
from objloader_simple import *
from geometry_msgs.msg import Pose2D
#from sensor_msgs.msg import Image

# TODO get pose data from joystick/simulink

# Gets proxy pose from simulink
# Publishes proxy pose to gazebo
# Publishes delayed proxy pose as slave pose to gazebo
class ARCalibration:

    def __init__(self, rate=30):
        rospy.init_node("ar_calibration")
        self.r = rospy.Rate(rate)
        # Load obj file to be projected
        self.obj = OBJ('/home/caghank/ck_ws/src/mobot_start/src/Models/mobot_angle_corrected.obj', swapyz=False)
        # Initialize camera
        self.cap = cv2.VideoCapture(1) # video device 0 for laptop cam, 1 for external cam
        self.alpha = 0.5
        self.calib_image = None
        self.img_init()
        
        self.calib_points = np.array([[2.0, 6.0], [2.0, 10.0], [-3.0, 11.0], [-1.0, 7.0]])
        # Offset values to match the virtual and the real robot positions
        self.offset_x = 0.0
        self.offset_y = 0.0
        # Scale value to match the virtual and the real workspace
        self.pose_scale_x = 1.03
        self.pose_scale_y = 1.0
        self.pose_scale_z = 1.0
        # Proxy pose holder
        self.proxy_pose = Pose2D()
        # Camera calibration matrix
        self.calib_mat = np.array([[706.156034, 0, 308.674187], [0, 707.000735, 251.985347], [0, 0, 1]])
        # Rotate in x_cam axis to match the perspective
        self.perspective_correction_ang = 45
        self.perspective_correction_mat = np.array([[1, 0, 0], [0, np.cos(np.deg2rad(self.perspective_correction_ang)), np.sin(np.deg2rad(self.perspective_correction_ang))], [0, -np.sin(np.deg2rad(self.perspective_correction_ang)), np.cos(np.deg2rad(self.perspective_correction_ang))]])
        # Matrix variables
        self.scale_mat = np.eye(3) * 2.3
        self.projection_mat = np.zeros(shape=(3, 4))


    def img_init(self):
        ret, frame = self.cap.read()
        if not ret:
            print ("Unable to capture video")
            return
        self.calib_image = frame

    def update(self):
        self.img_init()
        AR_background = self.calib_image.copy()

        

        for calib_point_num in range(4):
            x = self.calib_points[calib_point_num, 0]
            y = self.calib_points[calib_point_num, 1]
            self.proxy_pose.x = x
            self.proxy_pose.y = y

            self.calc_projection_matrix()
            self.calib_image = self.render(self.calib_image)

        transparent_img = cv2.addWeighted(AR_background, self.alpha, self.calib_image, 1 - self.alpha, 0)
        cv2.imshow('frame', transparent_img)
        #cv2.imshow('frame', self.calib_image)
        cv2.waitKey(1)

    # Renders the object on top of the camera image for given projection matrix
    def render(self, img):
        vertices = self.obj.vertices
        for face in self.obj.faces:
            face_vertices = face[0]
            points = np.array([vertices[vertex - 1] for vertex in face_vertices])
            #print(points)
            #print("-------------------")
            points = np.dot(points, self.scale_mat)
            dst = cv2.perspectiveTransform(points.reshape(-1, 1, 3), self.projection_mat)
            imgpts = np.int32(dst)
            
            cv2.fillConvexPoly(img, imgpts, (0, 0, 255, 255))
        return img

    def calc_projection_matrix(self):
        #---------- Single rotation approach ---------- 
        # Computationally cheaper
        # Get the rotation matrix from single rotation around z-axis
        rot_max = euler2mat(0.0, self.proxy_pose.theta, 0.0)


        #---------- Quaternion approach ----------
        # More compatible with ros geometry_msgs/Pose
        # Get rotation matrix from proxy orientation quaternion
        #rot_max = self.quat_to_rot_max()


        # Get translation vector from proxy position
        transl_vec = np.array([[-self.proxy_pose.x * self.pose_scale_x + self.offset_x, -1.6*self.pose_scale_z, -self.proxy_pose.y * self.pose_scale_y + self.offset_y]]) # prev: self.proxy_pose.position.x-y  
        # transl_vec_corrected = np.dot(self.perspective_correction_mat, transl_vec.T)
        # transl_vec = np.array([[0.0, -1.6, -8.0]])
        # Merge rotation matrix and translation colum vector into 3*4 [R|T] matrix
        rot_and_transl = np.concatenate((rot_max, transl_vec.T), axis=1) # .T
        # Calculate projection matrix from P = K * [R|T] K:calib_mat
        self.projection_mat = np.dot(self.calib_mat, rot_and_transl)
        

    
