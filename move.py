#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


positions = ()



def movebase_client(x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    
    print ("x: " + str(x))
    print ("y: " + str(y))
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =  x
    goal.target_pose.pose.position.y =  y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
    
        # Define a dictionary of predefined locations and their coordinates
        places = {"1": "centrale kruispunt","2": "Sam's huis (volledig links boven)","3": "parkingspot 2 (kleine parking linksachter)" ,"4": "parkingspot 1 (kleine parking linksachter)","5": "hospital","6": "thuis", "7": "politiekantoor", "9": "centraal boven", "10": "onder middenintersectie",  "12": "parkingspot 1 (kleine parking midden)",  "13": "parkingspot 2 (kleine parking midden)", "14": "parkingspot 1 (grote parking)", "15": "parkingspot 2 (grote parking)", "16": "parkingspot 3 (grote parking)", "17": "parkingspot 4 (grote parking)", "18": "centraal boven" }
        locations = {
    # politiekantoor = achterkant , grote parking = voorkant ! 
    "centrale kruispunt": [0.77859, -1.077846] , #[0.06667923, -0.239073623]
    "linkerarm kruispunt": [1.0797138214111, -1.077846], #, 0.7905325
    "Sam's huis (volledig links boven)": [-1.2997138214111, -0.7128434943 ],# [1.0797138214111, -2.048434943 ]
    " vóór de parkingspot 2 (kleine parking linksachter)": [1.0360345543, 2.0676534545],
    "parkingspot 2 (kleine parking linksachter)": [0.72214454, 2.15112667],
    " vóór de parkingspot 1 (kleine parking linksachter)": [1.022934435, 2.384958343],
    "parkingspot 1 (kleine parking linksachter)": [0.74489343, 2.3411932],
    "linker arm onder T kruis": [1.038435534, 1.52748903],
    "centrale punt onder T kruis": [0.0536745, 1.52748903],
    "hospital": [0.13513730369562, 0.98176345],
    "thuis": [0.3284343, 1.670032],
    "politiekantoor": [-0.53464543, -1.69711], # , -0.6029
    "rechterarm onder T kruis": [-1.14266, 1.52748903],
    " vóór de parkingspot 1 (kleine parking midden)": [-1.0270345543, 0.563534545],
    "parkingspot 1 (kleine parking midden)": [-0.80017, 0.62294] ,
    " vóór de parkingspot 2 (kleine parking midden)": [-1.0270345543, 0.401188343],
    "parkingspot 2 (kleine parking midden)": [-0.8222154, 0.3752667],
    "rechterarm kruispunt": [-1.1811, -0.25388], 
    " vóór de parkingspot 1 (grote parking)": [-1.0720345543, -1.21454545],
    "parkingspot 1 (grote parking)": [-0.80032, -1.21454545] ,
    " vóór de parkingspot 2 (grote parking)": [-1.0720345543, -1.421454545],
    "parkingspot 2 (grote parking)": [-0.80032, -1.421454545],
    " vóór de parkingspot 3 (grote parking)": [-1.0720345543, -1.6634342],
    "parkingspot 3 (grote parking)": [-0.80032, -1.6634342] ,
    " vóór de parkingspot 4 (grote parking)": [-1.0720345543, -1.8711843],
    "parkingspot 4 (grote parking)": [-0.80032, -1.8711843],
    "centraal boven": [-0.4743, 0.0304] , # , -0.9840345 [0.06667923, -2.0316543]
    }

        

        # Prompt the user to select a location
        for key in places.keys():
            print(key + " " +places[key])
               
        selection = input("Select the number of the location:")

        
        print ("you selected:")
        print (places[selection])
        
        rospy.init_node('movebase_client_py')
        result = movebase_client(locations[selection][0],locations[selection][1])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")