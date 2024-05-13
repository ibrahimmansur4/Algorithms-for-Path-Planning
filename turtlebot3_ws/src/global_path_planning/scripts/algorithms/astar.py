#! /usr/bin/env python

import rospy
from math import sqrt
from algorithms.neighbors import find_neighbors
from heapq import heappush, heappop

def euclidean_distance(index, goal_index, width):
    """ Heuristic Function for A Star algorithm"""
    index_x = index % width
    index_y = int(index / width)
    goal_x = goal_index % width
    goal_y = int(goal_index / width)

    return sqrt((index_x - goal_x) ** 2 + (index_y - goal_y) ** 2)

def astar(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
    '''
    Performs A Star shortest path algorithm search on a costmap with a given start and goal node
    '''

    # dict for mapping g costs (travel costs) to nodes
    g_costs = {start_index: 0}

    # dict for mapping parents to nodes
    parents = {}

    # priority queue for efficient exploration of nodes
    pq = [(0, start_index)]

    while pq:
        _, current_node = heappop(pq)

        # If current_node is the goal, exit the main loop
        if current_node == goal_index:
            break

        # Close current_node to prevent from visiting it again
        grid_viz.set_color(current_node, "pale yellow")

        # Get neighbors of current_node
        neighbors = find_neighbors(current_node, width, height, costmap, resolution)

        for neighbor_index, step_cost in neighbors:
            if neighbor_index in g_costs:
                # If a shorter path is found, update g_cost and parent
                new_cost = g_costs[current_node] + step_cost
                if new_cost < g_costs[neighbor_index]:
                    g_costs[neighbor_index] = new_cost
                    parents[neighbor_index] = current_node
                    heappush(pq, (new_cost + euclidean_distance(neighbor_index, goal_index, width), neighbor_index))
            else:
                # Discover a new node
                g_costs[neighbor_index] = g_costs[current_node] + step_cost
                parents[neighbor_index] = current_node
                heappush(pq, (g_costs[neighbor_index] + euclidean_distance(neighbor_index, goal_index, width), neighbor_index))

                # Optional: visualize frontier
                grid_viz.set_color(neighbor_index, 'orange')

    if goal_index not in parents:
        rospy.logwarn('AStar: No path found!')
        return []

    # Reconstruct path by working backwards from target
    shortest_path = [goal_index]
    node = goal_index
    while node != start_index:
        node = parents[node]
        shortest_path.append(node)

    rospy.loginfo('AStar: Done reconstructing path')

    return shortest_path[::-1]
