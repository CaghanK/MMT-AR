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
class MobotVR:

    def __init__(self, rate=30):
        rospy.init_node("virtual_reality")
        self.r = rospy.Rate(rate)
        # Load obj file to be projected
        self.obj = OBJ('/home/caghank/ck_ws/src/mobot_start/src/Models/mobot_angle_corrected.obj', swapyz=False)
        # Initialize camera
        self.cap = cv2.imread('/home/caghank/Thesis/Figures/exp_2.png',-1) # video device 0 for laptop cam, 1 for external cam
        # Transparency of the virtual model
        self.alpha = 1.0
        # Scale value to match the virtual and the real workspace
        self.pose_scale_x = 2.0 #default: 1.03
        self.pose_scale_y = 2.0 #default: 1.03
        self.pose_scale_z = 1.03
        # Offset values to match the virtual and the real robot positions
        self.offset_x = 0.50 * self.pose_scale_x # -2.5 - 0.53
        self.offset_y = 2.50 * self.pose_scale_y # -0.5 - 0.49
        # Proxy pose holder
        self.proxy_pose = Pose2D()
        # Camera calibration matrix
        self.calib_mat = np.array([[706.156034, 0, 308.674187], [0, 707.000735, 251.985347], [0, 0, 1]])
        # Rotate in x_cam axis to match the perspective
        self.perspective_correction_ang = 23.5
        self.perspective_correction_mat = euler2mat(self.perspective_correction_ang*np.pi/180, 0.0, 0.0)
        # Matrix variables
        self.scale_mat = np.eye(3) * 2.0 # default: 1.8
        #self.rot_mat = np.eye(3)
        #self.transl_vec = np.array([[0.0, 0.0, 0.0]]).T
        #self.rot_and_transl = np.concatenate((self.rot_mat, self.transl_vec), axis=1)
        self.projection_mat = np.zeros(shape=(3, 4))
        # Subscriber to get pose
        self.sub = rospy.Subscriber("/proxy/pose_nondelayed", Pose2D, self.callback, queue_size=1)
        # Publisher to send image (possibly to rviz)
        #self.pub = rospy.Publisher("/to_rviz/AR_img", Image, queue_size=1)

        """TEST STUFF"""
        self.color_red = (0, 0, 255)
        self.color_green = (0, 255, 0)
        self.color_blue = (255, 0, 0)
        self.color_yellow = (0, 234, 255)
        self.targets = [[-1.0, 0.0], [-1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]
        self.current_target = 0
        self.epsilon = 0.05

        self.flashing = False
        self.flash_counter = 0
        self.flash_duration = 60

        self.wait_counter = 0
        self.wait_duration = 60
        self.color = self.color_red

    # Subscriber callback
    def callback(self, msg_in):
        self.proxy_pose = msg_in

    """
    Status check:
    0 - Just started
    1 - First checkpoint completed
    2 - Second CP completed
    3 - Finished
    """
    def update_status(self):
        if self.current_target == len(self.targets):
            self.color = self.color_blue
            return

        current_target = self.targets[self.current_target]
        print(current_target)

        if self.check_if_inside(current_target):

            if self.wait_counter < self.wait_duration:
                self.wait_counter += 1
                self.color = self.color_yellow
            else:
                self.flashing = True
                self.current_target += 1

        else:
            self.wait_counter = 0
            self.color = self.color_red

        if self.flashing:

            if self.flash_counter < self.flash_duration:
                self.color = self.color_green
                self.flash_counter += 1
            else:
                self.color = self.color_red
                self.flash_counter = 0
                self.flashing = False

    def check_if_inside(self, target_point):
        distance_to_target_squared = ((self.proxy_pose.x - target_point[0])**2 + (self.proxy_pose.y - target_point[1])**2)
        if distance_to_target_squared < self.epsilon**2:
            return True
        else:
            return False

    def update(self):
        self.update_status()
        self.calc_projection_matrix()
        # Read new image from camera
        img = self.cap.copy()
        frame = self.render(img, self.color)
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    # Renders the object on top of the camera image for given projection matrix
    def render(self, img, color):
        vertices = self.obj.vertices
        for face in self.obj.faces:
            face_vertices = face[0]
            points = np.array([vertices[vertex - 1] for vertex in face_vertices])
            #print(points)
            #print("-------------------")
            points = np.dot(points, self.scale_mat)
            dst = cv2.perspectiveTransform(points.reshape(-1, 1, 3), self.projection_mat)
            imgpts = np.int32(dst)
            
            cv2.fillConvexPoly(img, imgpts, color)
        return img

    def calc_projection_matrix(self):
        #---------- Single rotation approach ---------- 
        # Computationally cheaper
        # Get the rotation matrix from single rotation around z-axis
        rot_max = euler2mat(self.perspective_correction_ang*np.pi/180, self.proxy_pose.theta, 0.0)


        #---------- Quaternion approach ----------
        # More compatible with ros geometry_msgs/Pose
        # Get rotation matrix from proxy orientation quaternion
        #rot_max = self.quat_to_rot_max()


        # Get translation vector from proxy position
        transl_vec = np.array([[(-self.proxy_pose.x * self.pose_scale_x) - self.offset_x , -3.2*self.pose_scale_z, (-self.proxy_pose.y * self.pose_scale_y) - self.offset_y]]) # prev: self.proxy_pose.position.x-y  
        #print(transl_vec)
        transl_vec_corrected = np.dot(self.perspective_correction_mat, transl_vec.T)
        # transl_vec = np.array([[0.0, -1.6, -8.0]])
        # Merge rotation matrix and translation colum vector into 3*4 [R|T] matrix
        rot_and_transl = np.concatenate((rot_max, transl_vec_corrected), axis=1) # .T
        # Calculate projection matrix from P = K * [R|T] K:calib_mat
        self.projection_mat = np.dot(self.calib_mat, rot_and_transl)
        

    
