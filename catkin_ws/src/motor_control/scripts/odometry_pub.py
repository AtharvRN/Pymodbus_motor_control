#!/usr/bin/env python3
# license removed for brevity

from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

wheel_seperation = 0.8
wheel_radius = 0.2
PI = 3.14159
vl = 0
vr = 0

def read_data(data):
    global vl
    global vr
    vl = data.x
    vr = data.y

rospy.init_node('odometry_publisher')
rospy.Subscriber('odom_data',Vector3,read_data)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0
vth = 0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
# compute odometry in a typical way given the velocities of the robot
    

    rospy.Subscriber('odom_data',Vector3,read_data)
    print(vl,vr)
    vx = PI*wheel_radius*(vl+vr)
    vth = PI*wheel_radius*(vl-vr)/(wheel_seperation/2)
    vy = 0
    dt = (current_time - last_time).to_sec()
    
    delta_x = vx*cos(th)*dt
    delta_y = vx*sin(th)*dt
    delta_th = vth*dt

    x += delta_x
    y += delta_y
    th += delta_th
    #th = (1 - alpha) * th 

    #if(th > 2*pi): th -= 2*pi
    #if(th < 0): th += 2*pi

   # print "fused_theta: " + str(th*57.32) + "  imu_theta: " + str(yaw*57.32) + "  wheel_theta: " + str(th_wheel*57.32)
    
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    
   # odom_quat_neg = tf.transformations.quaternion_from_euler(0, 0, -th)

    # first, we'll publish the transform over tf, not required as we are using pose_ekf's odometry
   # odom_broadcaster.sendTransform(
   #     (-x, -y, 0.0),
   #     odom_quat_neg,
   #     current_time,
   #     "wheel_frame",
   #     "base_link"
   # )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    
    if (vl == 0 and vr == 0):
        odom.pose.covariance[0] = 1e-9
        odom.pose.covariance[7] = 1e-3
        odom.pose.covariance[8] = 1e-9
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e-9
        odom.twist.covariance[0] = 1e-9
        odom.twist.covariance[7] = 1e-3
        odom.twist.covariance[8] = 1e-9
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 1e-9
    else:
      odom.pose.covariance[0] = 1e-3
      odom.pose.covariance[7] = 1e-3
      odom.pose.covariance[8] = 0.0
      odom.pose.covariance[14] = 1e6
      odom.pose.covariance[21] = 1e6
      odom.pose.covariance[28] = 1e6
      odom.pose.covariance[35] = 1e3
      odom.twist.covariance[0] = 1e-3
      odom.twist.covariance[7] = 1e-3
      odom.twist.covariance[8] = 0.0
      odom.twist.covariance[14] = 1e6
      odom.twist.covariance[21] = 1e6
      odom.twist.covariance[28] = 1e6
      odom.twist.covariance[35] = 1e3
    
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom_pub.publish(odom)
    #vel_pub.publish(Twist(Vector3(vel, 0, 0), Vector3(0, 0, vth)))
    last_time = current_time
    r.sleep()
