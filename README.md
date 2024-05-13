# Contents

Instructions to run the simulation are in the respective workspace directory

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make


##### docker related:

docker cp /path/to/local/folder container_id:/path/in/container

sudo apt install ros-noetic-nav-core
apt install ros-noetic-base-local-planner
apt-get install ros-noetic-navigation


roslaunch ros_world turtlebot3_world.launch
roslaunch global_path_planning turtlebot3_ros_world.launch


make the .py file executable

rosrun global_path_planning path_planning_server.py



## A star algorithm


### Reasons:
The A* algorithm is a popular path planning algorithm used in robotics applications. It is an informed search algorithm that efficiently finds the shortest path between a start and goal node by considering both the actual distance traveled (g-cost) and the estimated distance to the goal.


### For Report
The provided code implements the A* (A-star) algorithm for finding the shortest path between a given start and goal node on a 2D grid. The algorithm uses a combination of data structures and heuristic functions to efficiently explore the search space and find the optimal path.

Overview of the implementation:

Heuristic Function: The euclidean_distance function is used as the heuristic function for the A* algorithm. It calculates the Euclidean distance between the current node and the goal node, which serves as an admissible estimate of the actual cost to reach the goal.
Data Structures:
g_costs: A dictionary that stores the travel cost (g-cost) from the start node to each visited node.
parents: A dictionary that stores the parent of each visited node, allowing for the reconstruction of the shortest path.
pq: A priority queue (implemented using the heapq module) that stores the nodes to be explored, ordered by the sum of the g-cost and the heuristic cost.
Algorithm Flow:
The algorithm starts at the given start_index and initializes the g_costs and parents dictionaries.
The pq priority queue is initialized with the start node and its heuristic cost.
The main loop continues as long as the pq is not empty.
In each iteration, the node with the lowest total cost (g-cost + heuristic cost) is dequeued from the pq.
If the dequeued node is the goal node, the algorithm breaks out of the main loop.
Otherwise, the node is marked as "closed" in the grid_viz (optional visualization).
The neighbors of the current node are explored, and their g-costs and parent information are updated in the respective dictionaries. New nodes are added to the pq with their total cost (g-cost + heuristic cost).
After the goal node is found, the shortest path is reconstructed by working backwards from the goal node using the parents dictionary.
Usage:
The astar function takes the following parameters:
start_index: The index of the starting node.
goal_index: The index of the goal node.
width: The width of the grid.
height: The height of the grid.
costmap: The cost map of the grid, which represents the traversal cost of each node.
resolution: The resolution of the grid.
origin: The origin of the grid.
grid_viz: An optional visualization object for displaying the explored nodes.
The function returns the list of node indices representing the shortest path, or an empty list if no path is found.
The choice of data structures and algorithms in this implementation is well-suited for the A* algorithm:

Dictionaries (g_costs and parents): These dictionaries are used to efficiently store and update the g-costs and parent information for each visited node, allowing for quick lookup and updates during the search process.
Priority Queue (pq): The priority queue, implemented using the heapq module, provides an efficient way to explore the nodes in order of their total cost (g-cost + heuristic cost), ensuring that the algorithm focuses on the most promising paths first.
Heuristic Function (euclidean_distance): The Euclidean distance is a suitable heuristic function for this problem, as it provides an admissible estimate of the actual cost to reach the goal. This helps the A* algorithm to converge to the optimal solution more quickly.
The overall implementation follows the standard A* algorithm, with the use of appropriate data structures to store and update the necessary information during the search process. This approach ensures the efficiency and correctness of the algorithm, making it suitable for finding the shortest path in a 2D grid-based environment.


## Greedy search algorithm


### Reasons:
The provided code implements the Greedy Search algorithm for path planning in robotics applications. Greedy Search is a type of uninformed search algorithm that always chooses the next node that appears to be closest to the goal, without considering the long-term consequences of that choice.


While Greedy Search is a simpler and faster algorithm compared to A*, it may not always find the optimal path, as it does not take the actual cost of the path into account. The choice between Greedy Search and A* (or other path planning algorithms) depends on the specific requirements of the robotics application, such as the importance of finding the shortest path, the complexity of the environment, and the available computational resources.

### For Report
The provided code implements the Greedy algorithm for finding the shortest path between a given start and goal node on a 2D grid. The Greedy algorithm is a suboptimal algorithm that selects the next node to explore based solely on the heuristic function, without considering the actual cost of the path.

Overview of the implementation:

Heuristic Function: The euclidean_distance function is used as the heuristic function for the Greedy algorithm. It calculates the squared Euclidean distance between the current node and the goal node, which serves as a guide for selecting the next node to explore.
Data Structures:
parents: A dictionary that stores the parent of each visited node, allowing for the reconstruction of the shortest path.
pq: A priority queue (implemented using the heapq module) that stores the nodes to be explored, ordered by the heuristic cost.
Algorithm Flow:
The algorithm starts at the given start_index and initializes the parents dictionary.
The pq priority queue is initialized with the start node and its heuristic cost.
The main loop continues as long as the pq is not empty.
In each iteration, the node with the lowest heuristic cost is dequeued from the pq.
If the dequeued node is the goal node, the algorithm breaks out of the main loop.
Otherwise, the node is marked as "closed" in the grid_viz (optional visualization).
The neighbors of the current node are explored, and their parent information is updated in the parents dictionary. New nodes are added to the pq with their heuristic cost.
After the goal node is found, the shortest path is reconstructed by working backwards from the goal node using the parents dictionary.
Usage:
The greedy function takes the following parameters:
start_index: The index of the starting node.
goal_index: The index of the goal node.
width: The width of the grid.
height: The height of the grid.
costmap: The cost map of the grid, which represents the traversal cost of each node.
resolution: The resolution of the grid.
origin: The origin of the grid.
grid_viz: An optional visualization object for displaying the explored nodes.
The function returns the list of node indices representing the shortest path, or an empty list if no path is found.
The choice of data structures and algorithms in this implementation is well-suited for the Greedy algorithm:

Dictionary (parents): The dictionary is used to efficiently store and update the parent information for each visited node, allowing for the reconstruction of the shortest path.
Priority Queue (pq): The priority queue, implemented using the heapq module, provides an efficient way to explore the nodes in order of their heuristic cost, ensuring that the algorithm focuses on the nodes that appear closest to the goal.
Heuristic Function (euclidean_distance): The squared Euclidean distance is a suitable heuristic function for the Greedy algorithm, as it provides a simple and fast way to estimate the distance to the goal.
The overall implementation follows the standard Greedy algorithm, with the use of appropriate data structures to store and update the necessary information during the search process. This approach ensures the efficiency of the algorithm, but it is important to note that the Greedy algorithm is a suboptimal algorithm and may not always find the shortest path, especially in complex environments with obstacles or non-uniform costs.


## dijkstra algorithm


### Reasons:
The provided code implements Dijkstra's algorithm for path planning in robotics applications. Dijkstra's algorithm is a well-known informed search algorithm that finds the shortest path between a start and goal node by considering the actual distance traveled (g-cost).



The choice between Dijkstra's algorithm, Greedy Search, and A* (or other path planning algorithms) depends on the specific requirements of the robotics application, such as the importance of finding the optimal path, the complexity of the environment, and the available computational resources.


### For Report
The provided code implements Dijkstra's algorithm for finding the shortest path between a given start and goal node on a 2D grid. Dijkstra's algorithm is a classic shortest path algorithm that guarantees the optimal solution by considering the actual cost of the path.

Overview of the implementation:

Data Structures:
parents: A dictionary that stores the parent and the cost to reach each visited node, allowing for the reconstruction of the shortest path.
pq: A priority queue (implemented using the heapq module) that stores the nodes to be explored, ordered by their actual cost.
Algorithm Flow:
The algorithm starts at the given start_index and initializes the parents dictionary.
The pq priority queue is initialized with the start node and its cost (0).
The main loop continues as long as the pq is not empty.
In each iteration, the node with the lowest actual cost is dequeued from the pq.
If the dequeued node is the goal node, the algorithm breaks out of the main loop.
Otherwise, the node is marked as "closed" in the grid_viz (optional visualization).
The neighbors of the current node are explored, and their cost and parent information are updated in the parents dictionary. New nodes are added to the pq with their actual cost.
After the goal node is found, the shortest path is reconstructed by working backwards from the goal node using the parents dictionary.
Usage:
The dijkstra function takes the following parameters:
start_index: The index of the starting node.
goal_index: The index of the goal node.
width: The width of the grid.
height: The height of the grid.
costmap: The cost map of the grid, which represents the traversal cost of each node.
resolution: The resolution of the grid.
origin: The origin of the grid.
grid_viz: An optional visualization object for displaying the explored nodes.
The function returns the list of node indices representing the shortest path, or an empty list if no path is found.
The choice of data structures and algorithms in this implementation is well-suited for Dijkstra's algorithm:

Dictionary (parents): The dictionary is used to efficiently store and update the parent and cost information for each visited node, allowing for the reconstruction of the shortest path.
Priority Queue (pq): The priority queue, implemented using the heapq module, provides an efficient way to explore the nodes in order of their actual cost, ensuring that the algorithm focuses on the nodes with the lowest cost.
The overall implementation follows the standard Dijkstra's algorithm, with the use of appropriate data structures to store and update the necessary information during the search process. This approach ensures the efficiency and correctness of the algorithm, as Dijkstra's algorithm is guaranteed to find the shortest path in a weighted graph, provided that all edge weights are non-negative.

The key difference between this implementation and the previous A* and Greedy implementations is the use of the actual cost (cost variable) instead of a heuristic function to guide the search. Dijkstra's algorithm explores the nodes in order of their actual cost, ensuring that the optimal path is found, while the A* algorithm uses a heuristic function to guide the search and potentially find the optimal path faster.
