#! /usr/bin/env python

import rospy
from algorithms.neighbors import find_neighbors
from heapq import heappush, heappop

def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
    '''
    Performs Dijkstra's shortest path algorithm search on a costmap with a given start and goal node
    '''

    # dict for mapping parents to nodes
    parents = {}

    # priority queue for efficient exploration of nodes
    pq = [(0, start_index)]

    while pq:
        cost, current_node = heappop(pq)

        # If current_node is the goal, exit the main loop
        if current_node == goal_index:
            break

        # Close current_node to prevent from visiting it again
        grid_viz.set_color(current_node, "pale yellow")

        # Get neighbors of current_node
        neighbors = find_neighbors(current_node, width, height, costmap, resolution)

        for neighbor_index, step_cost in neighbors:
            new_cost = cost + step_cost
            if neighbor_index not in parents or new_cost < parents[neighbor_index][0]:
                # Discover a new node or update the cost if a shorter path is found
                parents[neighbor_index] = (new_cost, current_node)
                heappush(pq, (new_cost, neighbor_index))

                # Optional: visualize frontier
                grid_viz.set_color(neighbor_index, 'orange')

    if goal_index not in parents:
        rospy.logwarn('Dijkstra: No path found!')
        return []

    # Reconstruct path by working backwards from target
    shortest_path = [goal_index]
    node = goal_index
    while node != start_index:
        cost, node = parents[node]
        shortest_path.append(node)

    rospy.loginfo('Dijkstra: Done reconstructing path')

    return shortest_path[::-1]
