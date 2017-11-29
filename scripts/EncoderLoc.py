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
odom = SimplifiedOdometry() #published odometry 
begin = False

def getVelocities(motor):

    global odom
    global curr_time 
    curr_time=motor.header.stamp.secs
    dt=curr_time-old_time
    if begin is True:
	dt=0

    dtheta = ((float(1)/width) * (motor.left_speed - motor.right_speed )) * dt
    odom.x += (0.5 * (motor.left_speed + motor.right_speed) * math.cos(pose.theta)) * dt
    odom.y += (0.5 * (motor.left_speed + motor.right_speed) * math.sin(pose.theta)) * dt
    #pose.theta += dtheta
    #pose.theta = constrain(pose.theta)
    pub.publish(odom)
    
    # totX += pose.x
    # totY += pose.y
    old_time=curr_time

def getPose(data):
    odom.theta = np.deg2rad(data.orientation)

def constrain(rad):
    while (rad < -math.pi):
        rad += 2*math.pi
    while (rad > math.pi):
        rad -= 2*math.pi
    return rad



def init():
    rospy.init_node("EncoderLoc", anonymous=True)
    rospy.Subscriber('odometry', SimplifiedOdometry, getPose)
    rospy.Subscriber("encoder", Motor, getVelocities)
    

    global pub
    pub = rospy.Publisher("poseEncoder", SimplifiedOdometry, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
