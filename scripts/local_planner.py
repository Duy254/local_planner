#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from math import pow, atan2, sqrt, acos, atan
from arduino_msg.msg import Motor
from nav_msgs.msg import Odometry

way_number = 1;
realMode = "real"
simMode = "simulation"

#pose can either be assigned info from twist or a pose2D msg
class Pose:
    x = 0
    y = 0
    theta = 0       #pose specific (pose2D.theta)
    quatW = 0       #odometry specific (Odometry.pose.pose.quaternion.w)

class turtlebot():

    def __init__(self):
        # Creating our node,publisher and subscriber
        rospy.init_node('local_planner', anonymous=True)

        #self.pose_subscriber = None
        #self.pub_twist = None
        self.pub_motor = rospy.Publisher('motorSpeed', Motor, queue_size=10)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 10)  # add a publisher for gazebo
        self.pose = Pose()
        self.pose2D = Pose2D()
        self.odom = Odometry()

        self.rate = rospy.Rate(10)
        self.mode = rospy.get_param("~mode", "real")

        if self.mode == realMode:
            self.pose_subscriber = rospy.Subscriber('pose', Pose2D, self.callback)

        if self.mode == simMode:
             # subscribe to simulation instead need navmsg
            self.pose_subscriber = rospy.Subscriber('odom', Odometry, self.callback)

            #self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

        self.w = rospy.get_param("~base_width", 0.2)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.002)

    # Callback function implementing the pose value received
    def callback(self, data):

        if (self.mode == realMode):
            print "Callback"
            #self.pose2D = data
            self.pose.x = round(data.x, 6)
            self.pose.y = round(data.y, 6)
            self.pose.theta = round(data.theta, 6)

        if (self.mode == simMode):
            #self.odom = data
            self.pose.x = round(data.pose.pose.position.x, 6)
            self.pose.y = round(data.pose.pose.position.y, 6)
            self.pose.quatW = round(data.pose.pose.orientation.w, 6)

    def pubMotors(self, linearX, angularZ):
        goal_vel = Motor()
        goal_twist = Twist()

        rightVel = linearX + angularZ * self.w / 2
        leftVel = linearX - angularZ * self.w / 2
        goal_vel.left_speed = leftVel
        goal_vel.right_speed = rightVel
        self.pub_motor.publish(goal_vel)

        # if (self.mode == simMode): #publish twist for gazebo simulation
        goal_twist.linear.x = linearX
        goal_twist.angular.z = angularZ
        self.pub_twist.publish(goal_twist)

    def move2goal(self):
        global way_number
        p = rospy.get_param("/waypoints")
        strx = "/waypoints/1/x"
        stry = "/waypoints/1/y"

        strx = strx.replace("1", str(way_number))
        stry = stry.replace("1", str(way_number))

        goal_pose = Pose2D()
        goal_pose.x = rospy.get_param(strx)
        goal_pose.y = rospy.get_param(stry)

        # goal_vel = Motor()
        # goal_twist = Twist()

        # while not rospy.is_shutdown() and sqrt(pow((goal_pose.x - self.odom.pose.pose.position.x), 2) + pow((goal_pose.y - self.odom.pose.pose.position.y), 2)) >= 0.05:
        while not rospy.is_shutdown() and sqrt(
                        pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= 0.05:

            print "X: " + str(self.pose.x)
            print "Y: " + str(self.pose.y)
            print "theta: " + str( self.pose.theta)

            # Porportional Controller
            # linear velocity in the x-axis:
            linearx = 0.1 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            # linearx= 0.1* sqrt(pow((goal_pose.x - self.odom.pose.pose.position.x), 2) + pow((goal_pose.y - self.odom.pose.pose.position.y), 2))

            # angular velocity in the z-axis:
            if self.mode == simMode:
                angularz = -0.8 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - acos(self.pose.quatW)*2) # quaternion to angle

            if self.mode == realMode:
                angularz = -0.8 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)


            # Publishing left and right velocities
            self.pubMotors(linearx, angularz)
            # self.right = linearx + angularz * self.w / 2
            # self.left = linearx - angularz * self.w / 2
            #
            # goal_vel.left_speed = self.left
            # goal_vel.right_speed = self.right
            # self.pub_motor.publish(goal_vel)
            #
            # goal_twist.linear.x=linearx
            # goal_twist.angular.z=angularz
            # self.pub_twist.publish(goal_twist)

            self.rate.sleep()

        # Stopping our robot after the movement is over and no more waypoints to go to
        if (way_number >= len(p)):
            self.pubMotors(0, 0)

            # self.right = 0
            # self.left = 0
            #
            # goal_vel.left_speed = self.left
            # goal_vel.right_speed = self.right
            # self.pub_motor.publish(goal_vel)
            #
            # goal_twist.linear.x=0;
            # goal_twist.angular.z=0;
            # self.pub_twist.publish(goal_twist)

        else:
            way_number = way_number + 1
            turtlebot().move2goal()

        rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        x = turtlebot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
