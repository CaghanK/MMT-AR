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
class MobotAR:

    def __init__(self, rate=30):
        rospy.init_node("augmented_reality")
        self.r = rospy.Rate(rate)
        # Load obj file to be projected
        self.obj = OBJ('/home/caghank/ck_ws/src/mobot_start/src/Models/mobot_angle_corrected.obj', swapyz=False)
        # Initialize camera
        self.cap = cv2.VideoCapture(1) # video device 0 for laptop cam, 1 for external cam
        # Transparency of the virtual model
        self.alpha = 0.3
        # Offset values to match the virtual and the real robot positions
        self.offset_x = -2.5
        self.offset_y = 3.5 - 10.0
        # Scale value to match the virtual and the real workspace
        self.pose_scale_x = 2.0 #default: 1.03
        self.pose_scale_y = 2.0 #default: 1.03
        self.pose_scale_z = 1.03
        # Proxy pose holder
        self.proxy_pose = Pose2D()
        # Camera calibration matrix
        self.calib_mat = np.array([[706.156034, 0, 308.674187], [0, 707.000735, 251.985347], [0, 0, 1]])
        # Rotate in x_cam axis to match the perspective
        self.perspective_correction_ang = 45
        self.perspective_correction_mat = np.array([[1, 0, 0], [0, np.cos(np.deg2rad(self.perspective_correction_ang)), np.sin(np.deg2rad(self.perspective_correction_ang))], [0, -np.sin(np.deg2rad(self.perspective_correction_ang)), np.cos(np.deg2rad(self.perspective_correction_ang))]])
        # Matrix variables
        self.scale_mat = np.eye(3) * 2.3
        #self.rot_mat = np.eye(3)
        #self.transl_vec = np.array([[0.0, 0.0, 0.0]]).T
        #self.rot_and_transl = np.concatenate((self.rot_mat, self.transl_vec), axis=1)
        self.projection_mat = np.zeros(shape=(3, 4))
        # Subscriber to get pose
        self.sub = rospy.Subscriber("/proxy/pose_nondelayed", Pose2D, self.callback, queue_size=1)
        # Publisher to send image (possibly to rviz)
        #self.pub = rospy.Publisher("/to_rviz/AR_img", Image, queue_size=1)




    # Subscriber callback
    def callback(self, msg_in):
        self.proxy_pose = msg_in

    def update(self):
        self.calc_projection_matrix()
        # Read new image from camera
        ret, frame = self.cap.read()
        if not ret:
            print ("Unable to capture video")
            return 
        frame_bg = frame.copy()
        frame = self.render(frame)
        frame_combined = cv2.addWeighted(frame, self.alpha, frame_bg, 1 - self.alpha, 0)
        cv2.imshow('frame', frame_combined)
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
            
            cv2.fillConvexPoly(img, imgpts, (0, 0, 255)) #default: (0, 0, 255, 255)
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
        

    
