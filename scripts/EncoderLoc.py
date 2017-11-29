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
global begin 
begin= True

def getVelocities(motor):

    global odom
    global old_time
    global curr_time 
    global begin
    curr_time=motor.header.stamp.secs
    
    if begin is True:
	dt=0
        print "first dt"
    else:
        dt=curr_time-old_time

    dtheta = ((float(1)/width) * (motor.left_speed - motor.right_speed )) * dt
    odom.pose.x += (0.5 * (motor.left_speed + motor.right_speed) * math.cos(odom.orientation)) * dt
    odom.pose.y += (0.5 * (motor.left_speed + motor.right_speed) * math.sin(odom.orientation)) * dt
    #pose.theta += dtheta
    #pose.theta = constrain(pose.theta)
    pub.publish(odom)
    
    # totX += pose.x
    # totY += pose.y
    old_time=curr_time

def getPose(data):
    odom.orientation = np.deg2rad(data.orientation)

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
