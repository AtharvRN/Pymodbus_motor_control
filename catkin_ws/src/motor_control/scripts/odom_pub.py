import math
from math import sin, cos, pi
from motor_class import motor

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

wheel_seperation = 0.8
wheel_radius = 0.2
PI = 3.14159

class odom_pub:

    def __init__(self) :
        self.vl = 0
        self.vr = 0
        self.vx =0
        self.vy =0
        self.vth =0

        self.x = 0
        self.y = 0
        self.th = 0
