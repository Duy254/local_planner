#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from math import pow,atan2,sqrt

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('local_planner', anonymous=True)
	
      	self.pub_lmotor = rospy.Publisher('/cabot/leftvel', Float32, queue_size=10)
        self.pub_rmotor = rospy.Publisher('/cabot/rightvel', Float32, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/cabot/pose', Pose, self.callback)
        self.pose = Pose()

        self.rate = rospy.Rate(30)
	self.w = rospy.get_param("~base_width", 0.1)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.position.x = round(self.pose.position.x, 6)
        self.pose.position.y = round(self.pose.position.y, 6)
	

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.position.x), 2) + pow((goal_y - self.pose.position.y), 2))
        return distance

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.position.x = 40.443421
        goal_pose.position.y = -79.945592
        distance_tolerance = 0.00002
        vel_msg = Twist()
	

	#just added to hard code will need to remove once there is a topic to subsribe to that will continually update current pose 
	
	self.pose.position.x=40.443428 
	self.pose.position.y=-79.945562
	#self.pose.position.x=1 
	#self.pose.position.y=2
	self.pose.orientation.z=0

	#end hardcoded initial position

       	while not rospy.is_shutdown() and sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) + pow((goal_pose.position.y - self.pose.position.y), 2)) >= distance_tolerance:

		#Porportional Controller
		#linear velocity in the x-axis:
		linearx= 1.5 * sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) + pow((goal_pose.position.y - self.pose.position.y), 2))
		#vel_msg.linear.y = 0sour
		#vel_msg.linear.z = 0
		    

		#angular velocity in the z-axis:
		#vel_msg.angular.x = 0
		#vel_msg.angular.y = 0
		angularz = 4 * (atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x) - self.pose.orientation.z)
		    
		#Publishing left and right velocities

		self.right = 1.0 * linearx + angularz * self.w / 2 
		self.left = 1.0 * linearx - angularz * self.w / 2
		rospy.loginfo("publishing: (%f, %f)", self.right, self.left) 
		        
		self.pub_lmotor.publish(self.left)
		self.pub_rmotor.publish(self.right)


		self.rate.sleep()

      	#Stopping our robot after the movement is over
	self.right = 0
	self.left=0
	self.pub_lmotor.publish(self.left)
	self.pub_rmotor.publish(self.right)

	rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass

