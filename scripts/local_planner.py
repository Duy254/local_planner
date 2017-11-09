#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
#from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from math import pow,atan2,sqrt

way_number=1;

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('local_planner', anonymous=True)
	
      	self.pub_lmotor = rospy.Publisher('/leftvel', Float32, queue_size=10)
        self.pub_rmotor = rospy.Publisher('/rightvel', Float32, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/pose', Pose2D, self.callback) #2d pose 
        self.pose = Pose2D()

        self.rate = rospy.Rate(10)
	self.w = rospy.get_param("~base_width", 0.2)
	self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.002)



    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 6)
        self.pose.y = round(self.pose.y, 6)
	

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance
		
    #make a new function that changes the goal waypoint 
    #def choose_goal(self):
	
    
    #new function that makes the waypoints in parameters a list 
	

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
        #distance_tolerance = 0.0002
        vel_msg = Twist()


       	while not rospy.is_shutdown() and sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= self.distance_tolerance:

		#Porportional Controller
		#linear velocity in the x-axis:
		linearx= 1.5* sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
		    

		#angular velocity in the z-axis:
		angularz = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
		    
		#Publishing left and right velocities

		self.right = linearx + angularz*self.w / 2 
		self.left = linearx - angularz*self.w / 2
		#rospy.loginfo("publishing: (%s, %s)", strx,stry) 
		        
		self.pub_lmotor.publish(self.left)
		self.pub_rmotor.publish(self.right)


		self.rate.sleep()

      	#Stopping our robot after the movement is over and no more waypoints to go to
	
 	if (way_number >= len(p)):
		self.right=0
		self.left=0
		self.pub_lmotor.publish(self.left)
		self.pub_rmotor.publish(self.right)
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

