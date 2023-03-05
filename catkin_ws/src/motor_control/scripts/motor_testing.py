#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist,Vector3
from motor_class import motor
import time
from pymodbus.client.sync import ModbusSerialClient


# conversion from cmd_vel to wheel speeds 
# 1 rps - 2*pi degrees per second -> angular velocity = 
# vel_l = ((msg.linear.x - (msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
# vel_r = ((msg.linear.x + (msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
wheel_seperation = 0.8
wheel_radius = 0.2
PI = 3.14159

connect = motor.establish_connection()
left_motor = motor(unit = 1)
right_motor =  motor(unit = 2)
prev_vel_r = 0
prev_vel_l = 0
flag1 = False
flag2 = False

def callback(msg):
    global prev_vel_l
    global prev_vel_r
    global flag1,flag2
    print("cmd_vel",msg.linear.x)
    vel_l = ((msg.linear.x - (msg.angular.z * wheel_seperation / 2.0)) / wheel_radius) /(2*PI)
    vel_r = ((msg.linear.x + (msg.angular.z * wheel_seperation / 2.0)) / wheel_radius) /(2*PI)
    print("CURR",vel_l,vel_r)
    print("PREV",prev_vel_l,prev_vel_r)

    #if vel_l < 0.01 :
    #    left_motor.stop_jogging()
    #if vel_r < 0.01:
    #    right_motor.stop_jogging()
    
    if(prev_vel_l != vel_l or prev_vel_r != vel_r ) :
        left_motor.set_velocity(vel_l)
        right_motor.set_velocity(vel_r)
        #left_motor.start_jogging()
        #right_motor.start_jogging()
        flag2 = True
    
    if((not flag1) and flag2):
        left_motor.start_jogging()
        right_motor.start_jogging()
        flag1 = True
    

    

    #if not(vel_l <=0 and vel_r <=0) :
   

    #print(left_motor.get_velocity())
    #print(right_motor.get_velocity())
    prev_vel_l = vel_l
    prev_vel_r = vel_r

    
    
    #pub = rospy.Publisher('odom_data',Vector3 , queue_size=10)
    #v = Vector3()
    #v.x = left_motor.get_encoderVelocity()[0]
    #v.y = right_motor.get_encoderVelocity()[0]
    #v.z = 0
    #print(v.x,v.y)
    #pub.publish(v)
    
    #print(left_motor.get_encoderVelocity(),right_motor.get_encoderVelocity())
    #print(left_motor.get_velocity(),right_motor.get_velocity())
    

def utility():

    rospy.init_node('motor_controller', anonymous=True)
    #rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.Subscriber('cmd_vel',Twist , callback)
        #rate.sleep()
        # rospy.spin()
        pub = rospy.Publisher('odom_data',Vector3 , queue_size=10)
        v = Vector3()
        v.x = left_motor.get_encoderVelocity()
        v.y = right_motor.get_encoderVelocity()
        if(v.x > 5):
            v.x = 0
        if(v.y > 5):
            v.y = 0
        v.z = 0
        print(v.x,v.y)
        pub.publish(v)
        #time.sleep(0.01)

    left_motor.set_velocity(0)
    right_motor.set_velocity(0)
    left_motor.stop_jogging()
    right_motor.stop_jogging()




if(connect):
    utility()

else:
    print("Shutting Down")

print("Ending Program")