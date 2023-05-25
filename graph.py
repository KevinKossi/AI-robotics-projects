import math
import networkx as nx
import matplotlib.pyplot as plt
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Create a directed graph using NetworkX
G = nx.DiGraph()

def movebase_client(start,end):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    
    # Find the shortest path using Dijkstra's algorithm
    try:
        path = nx.dijkstra_path(G, start, end, weight="weight")
    except nx.NetworkXNoPath:
        return "No path found"

    # Move the Turtlebot along the path
    for i in range(len(path) - 1):
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = path[i][0]
        goal.target_pose.pose.position.y = path[i][1]
        goal.target_pose.pose.orientation.w = 1.0
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()


locations = {
    # politiekantoor = achterkant , grote parking = voorkant ! 
    "centrale kruispunt": [0.06667923, -0.239073623], #, 0.71693422 
    "linkerarm kruispunt": [1.0797138214111, -0.34981993675233], #, 0.7905325
    "Sam's huis (volledig links boven)": [1.0797138214111, -2.048434943 ],
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
    "centraal boven": [0.06667923, -2.0316543], # , -0.9840345
}

# Define a dictionary of predefined locations and their coordinates
places = {"1": "centrale kruispunt","2": "Sam's huis (volledig links boven)","3": "parkingspot 2 (kleine parking linksachter)" ,"4": "parkingspot 1 (kleine parking linksachter)","5": "hospital","6": "thuis", "7": "politiekantoor", "9": "centraal boven", "10": "onder middenintersectie",  "12": "parkingspot 1 (kleine parking midden)",  "13": "parkingspot 2 (kleine parking midden)", "14": "parkingspot 1 (grote parking)", "15": "parkingspot 2 (grote parking)", "16": "parkingspot 3 (grote parking)", "17": "parkingspot 4 (grote parking)", "18": "centraal boven" }

def euclidean_distance(coords1, coords2):
    x1, y1 = coords1
    x2, y2 = coords2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def find_path(G, start_point, end_point):
    # Convert the G to a NetworkX graph
    G = nx.Graph()
    for i in range(len(G)):
        for j in range(len(G[0])):
            if G[i][j] == 0:
                # Add a node for each empty space
                node_id = f"{i},{j}"
                G.add_node(node_id)
                # Add edges between adjacent empty spaces
                if i > 0 and G[i-1][j] == 0:
                    G.add_edge(node_id, f"{i-1},{j}")
                if j > 0 and G[i][j-1] == 0:
                    G.add_edge(node_id, f"{i},{j-1}")

    # Find the shortest path using Dijkstra's algorithm
    try:
        path = nx.shortest_path(G, start_point, end_point)
        nx.dijkstra_path()
        return path
    except nx.NetworkXNoPath:
        return None


# Convert the locations dictionary to a graph representation
graph = {

}

# Add nodes and edges to the graph
for loc, coords in locations.items():
    G.add_node(loc, pos=coords[:2])
    graph[loc] = []



for loc1, coords1 in locations.items():
    for loc2, coords2 in locations.items():
        if loc1 != loc2:
            distance = euclidean_distance(coords1[:2], coords2[:2])
            if distance <= 1.8:
                G.add_edge(loc1, loc2, weight=distance)
                graph[loc1].append(loc2)

# print(graph)
nx.draw(G, with_labels=True)
plt.show()

if __name__ == '__main__':
    # Define the initial start point
    start_point = "centrale kruispunt"

    while True:
        # Prompt the user to select a location
        for key in places.keys():
            print(key + " " +places[key])
            
        end_point = input("Select the number of new location:")

            # Exit the loop if the user types 'exit'
        if end_point.lower() == 'exit':
            break
            # Check if the end point is valid
        if end_point not in places:
            print("Invalid end point. Please try again.")
            continue


        print ("you selected:")
        print (places[end_point])

        
        # # Find the shortest path using the Dijkstra algorithm
        # path = find_path(G, start_point, end_point)

        shortest_path = nx.dijkstra_path(G,source= start_point, target= places[end_point] , weight="weight")
        movebase_client(start_point, places[end_point])

        start_point = places[end_point]
        
        print(f"Shortest path from {start_point} to {places[end_point]}: {shortest_path}")

        # Move the Turtlebot along the path
        # navigate_turtlebot(path)

        # Update the start point for the next iteration
