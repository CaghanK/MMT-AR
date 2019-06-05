#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from objloader_simple import *
from geometry_msgs.msg import Pose
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
        self.obj = OBJ('/home/caghank/ck_ws/src/mobot_start/src/Models/mobot.obj', swapyz=False)
        # Initialize camera
        self.cap = cv2.VideoCapture(0) # video device 0 for laptop cam, 1 for external cam
        # Proxy pose holder
        self.proxy_pose = Pose()
        # Camera calibration matrix
        self.calib_mat = np.array([[706.156034, 0, 308.674187], [0, 707.000735, 251.985347], [0, 0, 1]])
        # Rotate in x_cam axis to be realistic
        self.perspective_correction_ang = 45
        self.perspective_correction_mat = np.array([[1, 0, 0], [0, np.cos(np.deg2rad(self.perspective_correction_ang)), np.sin(np.deg2rad(self.perspective_correction_ang))], [0, -np.sin(np.deg2rad(self.perspective_correction_ang)), np.cos(np.deg2rad(self.perspective_correction_ang))]])
        # Matrix variables
        self.scale_mat = np.eye(3) * 2.5
        #self.rot_mat = np.eye(3)
        #self.transl_vec = np.array([[0.0, 0.0, 0.0]]).T
        #self.rot_and_transl = np.concatenate((self.rot_mat, self.transl_vec), axis=1)
        self.projection_mat = np.zeros(shape=(3, 4))
        # Subscriber to get pose
        self.sub = rospy.Subscriber("/sim/mobot_pose", Pose, self.callback, queue_size=1)
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
        frame = self.render(frame)
        cv2.imshow('frame', frame)
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
        # Get rotation matrix from proxy orientation quaternion
        rot_max = self.quat_to_rot_max()
        # Get translation vector from proxy position
        transl_vec = np.array([[-self.proxy_pose.position.x, -1.6, -self.proxy_pose.position.y]])
        # transl_vec_corrected = np.dot(self.perspective_correction_mat, transl_vec.T)
        # transl_vec = np.array([[0.0, -1.6, -8.0]])
        # Merge rotation matrix and translation colum vector into 3*4 [R|T] matrix
        rot_and_transl = np.concatenate((rot_max, transl_vec.T), axis=1) # .T
        # Calculate projection matrix from P = K * [R|T] K:calib_mat
        self.projection_mat = np.dot(self.calib_mat, rot_and_transl)

    def quat_to_rot_max(self):
        # Convert quaternion to rotation matrix using formula below
        '''
        1 - 2*qy^2 - 2*qz^2	    2*qx*qy - 2*qz*qw	    2*qx*qz + 2*qy*qw
        2*qx*qy + 2*qz*qw	    1 - 2*qx^2 - 2*qz^2	    2*qy*qz - 2*qx*qw
        2*qx*qz - 2*qy*qw	    2*qy*qz + 2*qx*qw	    1 - 2*qx^2 - 2*qy^2
        '''
        x = self.proxy_pose.orientation.x
        y = -self.proxy_pose.orientation.z
        z = self.proxy_pose.orientation.y
        w = self.proxy_pose.orientation.w
        """ x = 0.0
        y = 0.0
        z = 0.0
        w = 1.0 """

        rot_max = np.array([[1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w], [2*x*y*+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w], [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]])
        return rot_max
        

    
