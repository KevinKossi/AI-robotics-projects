#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
import networkx as nx

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

import math

locations = {...} # Your defined locations
graph = nx.Graph()

def get_distance(p, q):
    return sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def make_graph(locations):
    for loc1 in locations:
        for loc2 in locations:
            if loc1 != loc2:
                graph.add_edge(loc1, loc2, weight=get_distance(locations[loc1], locations[loc2]))

def callback(data):
    global current_position
    current_position = [data.pose.pose.position.x, data.pose.pose.position.y]

def movebase_client(x,y):
    # The implementation of this function would remain mostly same
    ...

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')

    global current_position

    odom_subscriber = rospy.Subscriber("odom", Odometry, callback)

    make_graph(locations)

    # Current position would be available after odom callback is called
    while current_position is None:
        rospy.sleep(1) # Wait for current position to be available. Continue once it is.

    # Calculate shortest paths from current position to all other positions
    shortest_paths = nx.single_source_dijkstra_path(graph, current_position, weight='weight')

    # The following code would mostly remain same
    ...
