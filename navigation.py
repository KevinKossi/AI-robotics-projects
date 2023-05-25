#!/usr/bin/env python
# license removed for brevity
import math
from maze import find_path
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
        # Given dictionary of of possible  locations [posX,posY,rotZ]
        locations = {
            "midden kruispunt": [-0.79517346620559699, -1.2317148447036743, 0.71693422 ],
            "linker middenintersectie": [-1.79713821411132, -1.30981993675233, 0.7905325],
            " devant rechter middenintersectie": [0.362673759468445, -0.94167077541351, 0.7731],
            "rechtermiddenperkingtje": [0.3980456,-1.78620, -0.993364],
            "rechtermiddenparking": [0.165139432, -1.80085094, -0.9952321],
            "linkermiddenparkingtje": [0.407557815, -1.96464764, -0.98415673],
            "linker middenparkingtje": [0.223648490695933, -2.0316543, -0.9840345],
            "politiekantoor": [0.429612934589388, -2.94235181808, -0.6029],
            "onder middenintersectie": [ -0.405922250227928, -3.11896824836736, 0.80756, 0.589],
            "hospital": [-0.55752730369562, -2.74529743194586],
            "rechter ondersectie": [0.710141539573665, -2.8292450904846, -0.9817],
            # "linker ondersectie": [],
            # "midden ondersectie": [],
            # "linker bovensectie": [],
            # "midden bovensectie": [],
            # "rechter bovensectie": [] 
        }

        def euclidean_distance(coords1, coords2):
            x1, y1 = coords1
            x2, y2 = coords2
            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)



        # Convert the locations dictionary to a graph representation
        graph = {

        }

        # Loop through the locations dictionary and add each location as a node in the graph
        for loc, coords in locations.items():
            graph[loc] = []

        # Loop through the locations dictionary again and add edges between neighboring locations
        for loc1, coords1 in locations.items():
            for loc2, coords2 in locations.items():
                if loc1 != loc2:
                    distance = euclidean_distance(coords1[:2], coords2[:2])  # Use only the x and y coordinates for distance calculation
                    if distance <= 1.3:  # Add an edge if the distance is less than or equal to 1.0 (you can adjust this threshold as needed)
                        graph[loc1].append(loc2)

        print(graph)

        # Define a dictionary of predefined locations and their coordinates
        places = {"1": "midden kruispunt","2": "linker middenintersectie","3": " devant rechter middenintersectie" ,"4": "rechtermiddenperkingtje","5": "rechtermiddenparking","6": "linkermiddenparkingtje", "7": "linker middenparkingtje","8": "politiekantoor","9": "onder middenintersectie", "10": "onder middenintersectie", "11": "hospital", "12": "rechter ondersectie" }

        # Define the initial start point
        start_point = "midden kruispunt"

        while True:
            # Prompt the user to select a location
            for key in places.keys():
                print(key + " " +places[key])
                
            end_point = input("Select the new location:")

                # Exit the loop if the user types 'exit'
            if end_point.lower() == 'exit':
                break
                # Check if the end point is valid
            if end_point not in locations:
                print("Invalid end point. Please try again.")
                continue

                # Convert the graph to a 2D array representing the maze
            maze = [[0 for _ in range(len(locations))] for _ in range(len(locations))]
            for i, loc1 in enumerate(locations.keys()):
                for j, loc2 in enumerate(locations.keys()):
                    if loc2 in graph[loc1]:
                        maze[i][j] = 1

            print ("you selected:")
            print (places[end_point])

            # dijkstra 
                # Convert the graph to a 2D array representing the maze
            maze = [[0 for _ in range(len(locations))] for _ in range(len(locations))]
            for i, loc1 in enumerate(locations.keys()):
                for j, loc2 in enumerate(locations.keys()):
                    if loc2 in graph[loc1]:
                        maze[i][j] = 1
    

                    
            # Find the shortest path using the Dijkstra algorithm
            path = find_path(maze, start_point, end_point)
                # Print the path
            
            print(f"Shortest path from {start_point} to {end_point}: {path}")

            # Move the Turtlebot along the path
            # navigate_turtlebot(path)

            # Update the start point for the next iteration
            start_point = end_point

            
            rospy.init_node('movebase_client_py')
            # move turtlebot to desired place 

            result = movebase_client(locations[end_point][0],locations[end_point][1])
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
