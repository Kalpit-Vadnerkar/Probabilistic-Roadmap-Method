# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import random
import math

disk_robot = True #(change this to False for part II) 


obstacles = None # the obstacles 
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap, 
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class  


def sample_positions(x_range, y_range, n_samples):
    """
    Returns a list of N uniformly sampled positions in a rectangular box
    defined by its range of x-positions and y-positions.
    """
    x_min, x_max = x_range
    y_min, y_max = y_range
    
    positions = []
    for i in range(n_samples):
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        positions.append((x, y))
    
    return positions


def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    
    robot_radius = max(robot_width, robot_height)/2.
    
    theta = 0
    # Uniformly sampled configurations
    points = sample_positions(x_limit, y_limit, n_samples=500)
      
    # the roadmap 
    graph = Roadmap()
    
    filtered_points = []
    for point in points:
        # configuration
        q = (point[0], point[1], theta)
        if not collision(q):
            filtered_points.append(point)
            graph.addVertex(q)
    vertices = graph.getVertices()
    for v in vertices:
        # Get nearest K configutation for a chosen configuration
        nearest = k_nearest_neighbors(graph, v.getConfiguration(), K=20)
        for neighbor in nearest:
            # Interpolate the two configurations 
            path = interpolate(v.getConfiguration(), neighbor[0].getConfiguration(), stepsize = 2)
            # Check the path for collision 
            is_path_valid = True
            for point in path:
                q = (point[0], point[1], theta)
                if collision(q):
                    is_path_valid = False
            # Collision-Free path found so add edge to graph.
            if is_path_valid:
                graph.addEdge(v, neighbor[0], neighbor[1], path) 
    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None
    
def find_path(q_start, q_goal, graph):
    path  = [] 
    
     # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

   
    return path   


# ----------------------------------------
# below are some functions that you may want to populate/modify and use above 
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances 
    """

    return None, None


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q. 
        You may also want to return the corresponding distances 
    """
    distances = []
    vertices = graph.getVertices()
    for v in vertices:
        if v.getConfiguration() != q:
            distances.append((v, distance(v.getConfiguration(), q)))
    distances.sort(key=lambda x: x[1])
    return [distances[i] for i in range(K)]

def distance (q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """
    return math.sqrt((q1[0] - q2[0]) ** 2 + (q1[1] - q2[1]) ** 2)

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.  
    """
    global obstacles, robot_width, robot_height
    robot = getRobotPlacement(q, robot_width, robot_height)
    robot_bb = {
        "x_min": min(p[0] for p in robot),
        "x_max": max(p[0] for p in robot),
        "y_min": min(p[1] for p in robot),
        "y_max": max(p[1] for p in robot)
    }
    
    # Check for collisions with each obstacle
    for obstacle in obstacles:
        if not (robot_bb["x_max"] < obstacle.x_min or
            robot_bb["x_min"] > obstacle.x_max or
            robot_bb["y_max"] < obstacle.y_min or
            robot_bb["y_min"] > obstacle.y_max):
            # The robot and obstacle overlap
            return True 
    # No collisions were found
    return False
   

def interpolate (q1, q2, stepsize=1):
    """
        Returns an interpolated local path between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """
    x1 = q1[0]
    y1 = q1[1]
    x2 = q2[0]
    y2 = q2[1]
    
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    
    # Determine whether the line is more horizontal or vertical
    is_horizontal = dx >= dy
    
    # Initialize starting and ending points
    if is_horizontal:
        x_start, y_start = min(x1, x2), y1
        x_end, y_end = max(x1, x2), y1
    else:
        x_start, y_start = x1, min(y1, y2)
        x_end, y_end = x1, max(y1, y2)
    
    # Initialize list of points
    points = [(x_start, y_start)]
    
    # Compute the next point on the line and add it to the list
    if is_horizontal:
        for x in range(x_start + stepsize, x_end + 1, stepsize):
            y = round(y_start + dy/dx*(x - x_start))
            points.append((x, y))
    else:
        for y in range(y_start + stepsize, y_end + 1, stepsize):
            x = round(x_start + dx/dy*(y - y_start))
            points.append((x, y))
    
    return points


if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
