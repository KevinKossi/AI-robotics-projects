#!/usr/bin/env python

#import the necessary library 
import rospy 
import numpy as np 
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry


# first bot = MamaEend => bringup bot 1 and give parameter [namespace: = mamaEend]
cmd_vel_topic = '/mamaEend/cmd_vel'
odom_topic = '/mamaEend/odom'
scan_topic = '/mamaEend/scan'
goal_topic = '/mamaEend/goal'
path_topic = '/mamaEend/path'

# other bots = eendje => bringup bots and give parameter [namespace: = eendjeEEN]
cmd_vel_topic = '/eendje/cmd_vel'
odom_topic = '/eendje/odom'
scan_topic = '/eendje/scan'
goal_topic = '/eendje/goal'
path_topic = '/eendje/path'



distance = 0.0

class Turtlebot:
    def __init__(self, name, namespace = '/'):
        self.name = name
        self.position = Point()
        self.distance_tolerance = 0.1
        self.obstacle_distance = 0.3
        self.obstacle = False
        self.publisher = rospy.Publisher(namespace + '/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber(namespace + '/base_pose_ground_truth', Odometry, self.update_position)
        self.scan_subscriber = rospy.Subscriber(namespace + '/base_scan', LaserScan, self.scan_callback)
        self.goal_subscriber = rospy.Subscriber(namespace + '/goal', Point, self.go_to_goal)
        self.rate = rospy.Rate(10)
        
     # deze functie zal uitgevoerd blijven , iedere keer dat er een neiuwe scan message is 
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    #lidar kan 360° scannen maar gaat enkel een array van 220 waarden geven vanwege de rotatiesnelheid
    nrValues = (len(data.ranges)) #ong 220 waarden 
    steps = (360/nrValues) # ong 1,6°
    
    # marges aanmaken voor het vooraanzicht 
    frontLeftMin = nrValues - 10
    frontRight = 10

    frontLeftDetector = min(tuple(filter(lambda item:item > 0.0, data.ranges[frontLeftMin:nrValues])))
    print ( f"frontleft: {frontLeftDetector}")
    frontRightDetector = min(tuple(filter(lambda item:item > 0.0, data.ranges[0:frontRight])))
    print (f" frontrigt:: {frontRightDetector}")

    twist.linear.y = 0.0
    twist.linear.z = 0.0
    def goForward() : 
        twist = Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        pub.publish(twist)
    def turnRight() : 
        twist = Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.linear.x = 0.05
        twist.angular.z = -0.5 # "+" is draaibeweging naar links, "-" naar rechts
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        pub.publish(twist)
        
    def turnLeft() : 
        twist = Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.linear.x = 0.05
        twist.angular.z = -0.5 # "+" is draaibeweging naar links, "-" naar rechts
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        pub.publish(twist)
    
    def goBack() : 
        twist = Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.linear.x = -0.15
        twist.angular.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        pub.publish(twist)
        
    if ( frontLeftDetector > 0.3 and frontRightDetector > 0.3 ) :

        goForward()
    if ( frontLeftDetector < 0.3 and frontRightDetector > 0.3 ) :

        turnRight()
    if ( frontLeftDetector > 0.3 and frontRightDetector < 0.3 ) :

        turnLeft()
    if ( frontLeftDetector < 0.2 and frontRightDetector < 0.2) :

        goBack()
    

    pub.publish(twist)
    


