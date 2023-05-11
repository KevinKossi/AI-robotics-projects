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
        locations = {"1": "Center of map","2": "Charging station","3": "Hospital"}
        coordinates = {"1":[0.000,-0.2846],"2":[-1.1284,-1.4429],"3":[1.1,2.2]}

        # Prompt the user to select a location
        for key in locations.keys():
            print(key + " " +locations[key])
               
        selection = input("Select the new location:")

        
        print ("you selected:")
        print (locations[selection])
        
        rospy.init_node('movebase_client_py')
        result = movebase_client(coordinates[selection][0],coordinates[selection][1])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


#locations 
{"1": {"x": -1.14633578, "y": -1.52145453}, "2": {"x":-0.794730453,"y":-1.2251879843},
 "3": {"x":-0.770730453,"y":-1.4129879843}, "4": {"x":-0.7750670453,"y":-1.6453279843}, "5": {"x":1.05335430453,"y":2.1439879843}, "6": {"x":0.765456313,"y":2.244128843},
 "7":{"x":0.74335430453,"y":2.074879843}, "8":{"x":0.05549330453,"y":-0.31280534}, "10":{"x":1.1977607443,"y":-0.31280534}, "13":{"x":1.06997607443,"y":-2.184140534},
 "12":{"x":0.0660142300,"y":-2.184140534}, "11":{"x":-1.1985142300,"y":-2.0404140534},"9": {" x ":-1.124354645,"y":-0.31280534},"17": {" x ":-1.019633445,"y":0.4204140534},
 "14": {"x":-1.11785142300, "y":1.5883940534} ,"15" : {"x":0.0508342435,"y":1.5872534},"16" : {"x":1.101967,"y":1.5057424} } 

#dict 
{"1":"Grote parking", "2":"GParking 1", "3": "GParking 2", "4" : "GParking 3", "5": "kleine parking", "6": "KParking 1", 
 "7": "KParking 2", "8" : " midpoint", "9": "left intersection", "10": "right intersection", "11":"bottom left ","12": "bottom center",
 "13": " bottom right", "14": " top left&", "15" : " top center& " ,"16": "top right&", "17": "empty 2 parking spots"}

#[posX,posY,rotZ, rotW]
{"mdden kruispunt": [-0.7951734662055969, -1.2317148447036743, 0.71693422 ],"linker middenintersectie": [-1.79713821411132, -1.30981993675233, 0.7905325],"rechter middenintersectie": [0.362673759468445, -0.94167077541351, 0.7731], "voor de rechtermiddenperkingtje": [0.3980456,-1.78620, -0.993364], "rechtermiddenparking": [0.165139432, -1.80085094, -0.9952321], "voor de linkermiddenparkingtje:": [0.407557815, -1.96464764, -0.98415673], "linker middenparkingtje": [0.223648490695933, -2.0316543, -0.9840345], "rechtero onderintersectie": [0.710141539573665, -2.8292450904846, -0.9817], "politiekantoor": [0.429612934589388, -2.94235181808, -0.6029], "onder middenintersectie": [ -0.405922250227928, -3.11896824836736, 0.80756, 0.589], "hospital": [-0.55752730369562, -2.74529743194586], "linker ondersectie": [] }