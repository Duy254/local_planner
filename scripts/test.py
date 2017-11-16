#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from arduino_msg.msg import Motor
from geometry_msgs.msg import Pose2D
import math 

pub = None
width = .254 #width of CaBot

# totX = 0
# totY = 0
totTheta = 0
dt = 0.1
pose = Pose2D() #2D messages

def getVelocities(motor):
    global pose
    dtheta = ((float(1)/width) * (motor.left_speed - motor.right_speed )) * dt
    pose.x += (0.5 * (motor.left_speed + motor.right_speed) * math.cos(pose.theta)) * dt
    pose.y += (0.5 * (motor.left_speed + motor.right_speed) * math.sin(pose.theta)) * dt
    pose.theta += dtheta
    pose.theta = constrain(pose.theta)
    #print "FUCK ROS"
    pub.publish(pose)
    
    # totX += pose.x
    # totY += pose.y

def constrain(rad):
    while (rad < -math.pi):
        rad += 2*math.pi
    while (rad > math.pi):
        rad -= 2*math.pi
    return rad

def init():
    rospy.init_node("fake_localizer", anonymous=True)
    rospy.Subscriber("motorSpeed", Motor, getVelocities)
    global pub
    pub = rospy.Publisher("pose", Pose2D, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
