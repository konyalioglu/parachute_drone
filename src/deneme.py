import numpy as np
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, SetJointProperties
from std_msgs.msg import Float64MultiArray, Float32
import math
import sys


class deneme:

    def __init__(self):
        self.motor_input = Float64MultiArray() 
        self.control_surface_input = Float64MultiArray()
        rospy.init_node('deneme_node')
        self.pub_motor = rospy.Publisher('/parachute_drone/vel_cmd', Float64MultiArray, queue_size=4)
        
        
    def control_loop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.motor_input.data = [5, 200]
            self.pub_motor.publish(self.motor_input)
            r.sleep()
        	      
        	      
if __name__ == '__main__':	
        de = deneme()
        de.control_loop()
