import numpy as np
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, SetJointProperties
from std_msgs.msg import Float64MultiArray, Float32
import math
import sys


class Scenario:

    def __init__(self):
		
        rospy.init_node('init_scenario')
        self.model_name_ = 'parachute_drone'


    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z
    
    
    def euler_angles_to_quaternions(self, phi, theta, psi):

        w = np.cos(phi * 0.5) * np.cos(theta * 0.5) * np.cos(psi * 0.5) \
            + np.sin(phi * 0.5) * np.sin(theta * 0.5) * np.sin(psi * 0.5)
        x = np.sin(phi * 0.5) * np.cos(theta * 0.5) * np.cos(psi * 0.5) \
            - np.cos(phi * 0.5) * np.sin(theta * 0.5) * np.sin(psi * 0.5)
        y = np.cos(phi * 0.5) * np.sin(theta * 0.5) * np.cos(psi * 0.5) \
            + np.sin(phi * 0.5) * np.cos(theta * 0.5) * np.sin(psi * 0.5)
        z = np.cos(phi * 0.5) * np.cos(theta * 0.5) * np.sin(psi * 0.5) \
            - np.sin(phi * 0.5) * np.sin(theta * 0.5) * np.cos(psi * 0.5)
        
        return w, x, y, z
    
    
    def body2earth_rate_transformation(self, angles, rates):
        vec1 = np.array([[1, \
                          np.tan(angles[1,0]) * np.sin(angles[0,0]), \
                          np.cos(angles[0,0]) * np.tan(angles[1,0])]])
    
        vec2 = np.array([[0, \
                          np.cos(angles[0,0]), \
                          -np.sin(angles[0,0])]])
    
        vec3 = np.array([[0, \
                          1/np.cos(angles[1,0]) * np.sin(angles[0,0]), \
                          1/np.cos(angles[1,0]) * np.cos(angles[0,0])]])
    
        transformation_matrix = np.concatenate((vec1, vec2, vec3), axis = 0)
        
        vec = transformation_matrix @ rates
        
        phi_dot = vec[0,0]
        theta_dot = vec[1,0]
        psi_dot = vec[2,0]
        
        return phi_dot, theta_dot, psi_dot
    
    
    def euler_rate2body_rate(self, angles, rates):
        vec1 = np.array([[1, 0,  -np.sin(angles[1,0])]])
    
        vec2 = np.array([[0, np.cos(angles[0,0]), np.sin(angles[0,0]) * np.cos(angles[1,0])]])
    
        vec3 = np.array([[0, -np.sin(angles[0,0]), np.cos(angles[0,0]) * np.cos(angles[1,0])]])
    
        transformation_matrix = np.concatenate((vec1, vec2, vec3), axis = 0)
        
        vec = transformation_matrix @ rates
        
        p = vec[0,0]
        q = vec[1,0]
        r = vec[2,0]
        
        return np.array([[p],[q],[r]])
    
    
    def body2earth_transformation(self, angles, vector):
        rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]])
        roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]])
        rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
        return np.matrix.transpose(rotz @ roty @ rotx) @ vector


    def earth2body_transformation(self, angles, vector):
        rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]])
        roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]])
        rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
        return rotz @ roty @ rotx @ vector
		   
    
    
    def locate(self):
        
        w, x, y, z = self.euler_angles_to_quaternions(0.0, 0.0, 0.0)

        state_msg = ModelState()
        state_msg.model_name = self.model_name_
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 50

        state_msg.pose.orientation.x = x
        state_msg.pose.orientation.y = y
        state_msg.pose.orientation.z = z
        state_msg.pose.orientation.w = w

        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0

        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0


        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException:
            print ("Service call failed: %s")

        return 0
    


if __name__ == '__main__':
    	
        sc = Scenario()
        sc.locate()

                
        print('Gazebo is Ready')

	
