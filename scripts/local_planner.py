#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import *
from math import *
from arduino_msg.msg import Motor
from nav_msgs.msg import Odometry
from navcog_msg.msg import SimplifiedOdometry
from hci_node.msg import waypoint

way_number = 1
realMode = "real" #operating on actual
simMode = "simulation" #simulating in gazebo
maxSpeed = 0.3

#PID constants

Kp = 0.4
Ki = 0.0
Kd = .4

#pose can either be assigned info from twist or a pose2D msg
class Pose:
    x = 0
    y = 0
    theta = 0       #pose specific (pose2D.theta)
    quatW = 0       #odometry specific (Odometry.pose.pose.quaternion.w)
    vel = 0         #average of 2 wheel velocities from Navcog


class turtlebot():

    begin = False

    def __init__(self):
        # Creating our node,publisher and subscriber
        rospy.init_node('local_planner', anonymous=True)
        self.wPoints = rospy.get_param("/waypoints")
        self.pub_motor = rospy.Publisher('motorSpeed', Motor, queue_size=10)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 10)  # add a publisher for gazebo
        self.pub_wayPts = rospy.Publisher('waypt', waypoint, queue_size=10)
        self.pose = Pose()
        #self.pose2D = Pose2D() #message
        self.odom = SimplifiedOdometry()

        # PID variables
        self.lastError = 0
        self.integral = 0

        self.rate = rospy.Rate(10)
        self.mode = rospy.get_param("~mode", "real")

        if self.mode == realMode:
            #self.pose_subscriber = rospy.Subscriber('pose', Pose2D, self.callback)
            #self.poseSubscriber = rospy.Subscriber('odometry', SimplifiedOdometry, self.getPose)
            self.poseSubscriber = rospy.Subscriber('poseEncoder', SimplifiedOdometry, self.getPose)
            self.PIDsubscriber = rospy.Subscriber('localPID', Vector3, self.tunePID)

        if self.mode == simMode:
             # subscribe to simulation instead need navmsg
            self.poseSubscriber = rospy.Subscriber('odom', Odometry, self.getPose)

        self.w = rospy.get_param("~base_width", 0.2)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.2)

    # Callback function implementing the pose value received
    def getPose(self, data):
        self.begin = True;
        #print "Callback"
        if self.mode == realMode:
            #self.pose2D = data
            self.pose.vel = data.speed
            self.pose.x = round(data.pose.x, 6)
            self.pose.y = round(data.pose.y, 6)
            self.pose.theta = np.deg2rad(data.orientation)
            self.pose.theta = self.constrain(self.pose.theta, -pi, pi)

        if self.mode == simMode:
            self.pose.x = round(data.pose.pose.position.x, 6)
            self.pose.y = round(data.pose.pose.position.y, 6)
            self.pose.quatW = round(data.pose.pose.orientation.w, 6)

    #Callback used for tuning PID params. Called when user publishes PID params to topic
    def tunePID(self, params):
        global Kp, Ki, Kd
        Kp = params.x
        Ki = params.y
        Kd = params.z
        print "\nParameters set to: Kp = ", Kp, " Ki = ", Ki, "Kd = ", Kd, "\n"

    #perform calculations to publish left and right velocities
    def pubMotors(self, linearX, angularZ):
        goal_vel = Motor() #motor message
        goal_twist = Twist() #twist message

        rightVel = linearX + angularZ * self.w / 2
        leftVel = linearX - angularZ * self.w / 2
        goal_vel.left_speed = leftVel
        goal_vel.right_speed = rightVel
        self.pub_motor.publish(goal_vel)

        if self.mode == simMode: #publish twist for gazebo simulation
            goal_twist.linear.x = linearX
            goal_twist.angular.z = angularZ
            self.pub_twist.publish(goal_twist)

    # constrain angl between lowBound and hiBound.
    # This function assumes everything is in radians
    def constrain(self, angl, lowBound, hiBound):
        while angl < lowBound:
            angl += 2*pi
        while angl > hiBound:
            angl -= 2*pi
        return angl

    def move2goal(self):
        global way_number

        point = self.wPoints[str(way_number)] #get current point from waypoints dict
        goal_pose = Pose2D()
        goal_pose.x = point["x"]
        goal_pose.y = point["y"]


        # added by Chris
        # if way_number == 1:
        #     dist = abs(goal_pose.y - self.pose.y);
        # elif way_number == 2:
        #     dist = abs(goal_pose.x - self.pose.x);
        # elif way_number == 3:
        #     dist = abs(goal_pose.y - self.pose.y);
        # elif way_number == 4:
        #     dist = abs(goal_pose.x - self.pose.x);
        # end added by Chris
        

        dist = sqrt((goal_pose.x - self.pose.x) ** 2 + (goal_pose.y - self.pose.y) ** 2)

        while not self.begin:
            pass

        while not rospy.is_shutdown() and dist >= self.distance_tolerance:

            # Porportional Controller
            # linear velocity in the x-axis:
            # linearx = 0.02 * sqrt((goal_pose.x - self.pose.x)**2 + (goal_pose.y - self.pose.y)**2)
            # linearx= 0.1* sqrt(pow((goal_pose.x - self.odom.pose.pose.position.x), 2) + pow((goal_pose.y - self.odom.pose.pose.position.y), 2))

            # angular velocity in the z-axis:
            # goes from [-pi, pi],
            linearx = maxSpeed
            angularz = 0

            #PD control
            goalAngle = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
            if self.mode == realMode:
                error = self.constrain(goalAngle - self.pose.theta, -pi, pi)

                if abs(self.pose.vel) <= 0.01 :
                    self.integral += error

                derivative = error - self.lastError
                self.lastError = error
                angularz = Kp * error + Ki * self.integral + Kd * derivative

            elif self.mode == simMode:
                angularz = -0.8 * self.constrain(goalAngle - acos(self.pose.quatW)*2, -pi, pi) # quaternion to angle

            #Publish info to waypt topic for HCI node
            wayMsg = waypoint()
            wayMsg.pointNum = way_number
            wayMsg.dist = sqrt((goal_pose.x - self.pose.x) ** 2 + (goal_pose.y - self.pose.y) ** 2)

            self.pub_wayPts.publish(wayMsg)

            print ("Current: {}, Desired: {}".format(np.rad2deg(self.pose.theta), np.rad2deg(goalAngle)))
            # print("angularz: {}".format(angularz))
            print "waypoint:", way_number, "\t dist to waypoint: ", dist #"point: ", goal_pose.x, goal_pose.y
            #print

            # # added by Chris
            # if way_number == 1:
            #     dist = abs(goal_pose.y - self.pose.y);
            # elif way_number == 2:
            #     dist = abs(goal_pose.x - self.pose.x);
            # elif way_number == 3:
            #     dist = abs(goal_pose.y - self.pose.y);
            # elif way_number == 4:
            #     dist = abs(goal_pose.x - self.pose.x);
            # # end added by Chris

            # Publishing left and right velocities
            dist = sqrt((goal_pose.x - self.pose.x) ** 2 + (goal_pose.y - self.pose.y) ** 2)
            self.pubMotors(linearx, angularz)
            self.rate.sleep()

        # Stopping our robot after the movement is over and no more waypoints to go to
        way_number += 1
        if way_number >= len(self.wPoints) + 1:
            print "stopped"
            self.pubMotors(0, 0)
        else:
            #TODO: make this iterative
            turtlebot().move2goal()

        rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        x = turtlebot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
