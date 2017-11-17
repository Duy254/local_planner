#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from math import pow,atan2,sqrt,acos, atan, pi
from arduino_msg.msg import Motor
from nav_msgs.msg import Odometry

way_number=1;

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('local_planner', anonymous=True)

	self.pub_motor=rospy.Publisher('/motorSpeed', Motor, queue_size=10)
      
	#subscribe to simulation instead need navmsg
	self.pose_subscriber=rospy.Subscriber('/odom', Odometry, self.callback) #Odom

	#add a publisher for gazebo 
	self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #self.pose = Pose2D()
	self.odom=Odometry()

        self.rate = rospy.Rate(10)
	self.w = rospy.get_param("~base_width", 0.2)
	#self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.002)
	self.distance_tolerance=0.05

    #Callback function implementing the pose value received
    def callback(self, data):
        self.odom = data
        self.odom.pose.pose.position.x = round(self.odom.pose.pose.position.x, 6)
        self.odom.pose.pose.position.y = round(self.odom.pose.pose.position.y, 6)

    def limitVel(linearV):
        if(linearV>5):
		linearV=5;
	if(linearV<-5):
		linearV=-5
    		
    		

    def move2goal(self):
	
	global way_number
	p=rospy.get_param("/waypoints")
	strx="/waypoints/1/x"
	stry="/waypoints/1/y"

	strx=strx.replace("1",str(way_number))
	stry=stry.replace("1",str(way_number))

        goal_pose = Pose2D()	
        goal_pose.x = rospy.get_param(strx)
        goal_pose.y = rospy.get_param(stry)    

	goal_vel = Motor()

	goal_twist=Twist()

	if (way_number<len(p)):
		self.distance_tolerance=0.8

       	while not rospy.is_shutdown() and sqrt(pow((goal_pose.x - self.odom.pose.pose.position.x), 2) + pow((goal_pose.y - self.odom.pose.pose.position.y), 2)) >= self.distance_tolerance:
	
		#Porportional Controller
		#linear velocity in the x-axis:
		linearx= 0.05* sqrt(pow((goal_pose.x - self.odom.pose.pose.position.x), 2) + pow((goal_pose.y - self.odom.pose.pose.position.y), 2))
			    
		#angular velocity in the z-axis:
		
		simangle=acos(self.odom.pose.pose.orientation.w)*2  # quaternion to angle
		goalangle=atan2(goal_pose.y - self.odom.pose.pose.position.y, goal_pose.x - self.odom.pose.pose.position.x)
		
		if(simangle>2*pi):
			simangle=2*pi
		if (simangle>pi):
			simangle-=2*pi
			
		angularz = -0.8* (goalangle-simangle) 
	
		#Publishing left and right velocities
		self.right = linearx + angularz*self.w / 2 
		self.left = linearx - angularz*self.w / 2

		goal_vel.left_speed=simangle
		goal_vel.right_speed=atan2(goal_pose.y - self.odom.pose.pose.position.y, goal_pose.x - self.odom.pose.pose.position.x) 
		self.pub_motor.publish(goal_vel)
		
		goal_twist.linear.x=linearx
		goal_twist.angular.z=angularz
		self.pub_twist.publish(goal_twist)

		self.rate.sleep()

      	#Stopping our robot after the movement is over and no more waypoints to go to
	
 	if (way_number >= len(p)):
		self.right=0
		self.left=0

		goal_vel.left_speed=self.left
		goal_vel.right_speed=self.right
		self.pub_motor.publish(goal_vel)

		goal_twist.linear.x=0;
		goal_twist.angular.z=0;
		self.pub_twist.publish(goal_twist)
		
	else: 
		way_number = way_number+1
		turtlebot().move2goal()

	rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass

