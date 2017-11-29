#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from arduino_msg.msg import Motor
from geometry_msgs.msg import Pose2D
from navcog_msg.msg import SimplifiedOdometry
import math 


pub = None
width = .2 #width of CaBot in meters

# totX = 0
# totY = 0
totTheta = 0
dt = 0.1
odom = SimplifiedOdometry() #2D messages

def getVelocities(motor):
    global pose
    dtheta = ((float(1)/width) * (motor.left_speed - motor.right_speed )) * dt
    pose.x += (0.5 * (motor.left_speed + motor.right_speed) * math.cos(pose.theta)) * dt
    pose.y += (0.5 * (motor.left_speed + motor.right_speed) * math.sin(pose.theta)) * dt
    pose.theta += dtheta
    pose.theta = constrain(pose.theta)
    pub.publish(pose)
    
    # totX += pose.x
    # totY += pose.y
def getIMU(imu):
    global

def constrain(rad):
    while (rad < -math.pi):
        rad += 2*math.pi
    while (rad > math.pi):
        rad -= 2*math.pi
    return rad

self.poseSubscriber = rospy.Subscriber('odometry', SimplifiedOdometry, self.getPose)

def init():
    rospy.init_node("encoderLoc", anonymous=True)
    rospy.Subscriber("motorSpeed", Motor, getVelocities)
    rospy.Subscriber("imu", Vector3Stamped, getIMU)
    global pub
    pub = rospy.Publisher("pose", Pose2D, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
