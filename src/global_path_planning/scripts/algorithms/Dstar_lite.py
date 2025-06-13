
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math
import time
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionResult
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Class structures
class Node:
    def __init__(self):
        self.is_obstacle = False
        self.parent = None
        self.neighbours = {}
        self.on_path = False
        self.g_cost = 999999  # should be infinite
        self.rhs = 999999  # should be infinite

class Graph:
    # graph constructor that creates an empty dictionary
    # nodes = {(x,y):Node} where x,y are coordinates of node
    # open_list = {(x,y): key}
    def __init__(self):
        self.nodes = {}
        self.open_list = {}
        self.obstacle_space = set()
        self.current_path = []
        self.open_length = 0
        self.map_data = None
        self.map_subscriber =None
        self.callback_check = False
    # loop through image and create node object for each pixel
    def create_nodes(self):
        for x in range(-304, 304):
            for y in range(-192, 192):
                y = -y
                if (x,y) not in self.obstacle_space:
                    self.nodes[(x, y)] = Node()

    # for given pixel and find it's neighbours
    def calculate_neighbours(self, curr_node):
        x = curr_node[0]
        y = curr_node[1]
        dig = 1
        strght = 1
        if (x-1,y+1) not in self.obstacle_space and x-1 >= -304 and y+1 < 192:
            if (x-1,y+1) not in self.open_list and self.nodes[(x-1,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x-1,y+1)] = dig
        if (x,y+1) not in self.obstacle_space and y+1 < 192:
            if (x,y+1) not in self.open_list and self.nodes[(x,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x,y+1)] = strght
        if (x+1,y+1) not in self.obstacle_space and x+1 < 304 and y+1 < 192:
            if (x+1,y+1) not in self.open_list and self.nodes[(x+1,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x+1,y+1)] = dig
        if (x-1,y-1) not in self.obstacle_space and x-1 >= -304 and y-1 >= -192:
            if (x-1, y-1) not in self.open_list and self.nodes[(x-1,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x-1, y-1)] = dig
        if (x,y-1) not in self.obstacle_space and y-1 >= -192:
            if (x, y-1) not in self.open_list and self.nodes[(x,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x, y-1)] = strght
        if (x+1,y-1) not in self.obstacle_space and x+1 < 304 and y-1 >= -192:
            if (x+1,y-1) not in self.open_list and self.nodes[(x+1,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x+1, y-1)] = dig
        if (x-1,y) not in self.obstacle_space and x-1 >= -304:
            if (x-1,y) not in self.open_list and self.nodes[(x-1,y)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x-1,y)] = strght
        if (x+1,y) not in self.obstacle_space and x+1 < 304:
            if (x+1,y) not in self.open_list and self.nodes[(x+1,y)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x+1,y)] = strght
    # Check if node is consistent
    def node_is_consistent(self,node):
        if self.nodes[node].g_cost == self.nodes[node].rhs:
            return True
        else:
            return False
    # Get heuristic distance
    def h(self,node,start):
        return np.sqrt((node[0] - start[0])**2 + (node[1] - start[1])**2)

    # Get key for sorting the open_list:
    def get_key(self,node, start):
        key = min(self.nodes[node].g_cost, self.nodes[node].rhs) + 0.9 * self.h(node,start)
        return key
    # Get smallest element from the open_list:
    def get_smallest(self,open_list):
        smallest = 9999999
        smallest_node = (-9999,-9999)
        for key, value in open_list.items():
            if open_list[key] < smallest:
                smallest = value
                smallest_node = key
        return smallest_node
    # D* Lite algorithm to find the shortest path
    def d_star_lite_algo(self, rob_x, rob_y, goal_x, goal_y,bg=None):
        # get coordinates for the start node
        start_node = (rob_x, rob_y)
        # get coordinates for the goal node
        goal_node = (goal_x, goal_y)
        # make cost of start node zero
        self.nodes[goal_node].rhs = 0
        self.open_list[goal_node] = 0  # key needs to be written here
        self.open_length += 1
        curr_node = goal_node
        while not curr_node == start_node and not len(self.open_list) == 0:
            # print("Curr",curr_node)
            # make g_cost = rhs
            self.nodes[curr_node].g_cost = self.nodes[curr_node].rhs
            # remove curr_node from the open list
            del self.open_list[curr_node]
            self.open_length -= 1
            # get successors of the curr_node
            self.calculate_neighbours(curr_node)
            for n in self.nodes[curr_node].neighbours:
                # bg[505 - n[1], n[0] + 555] = green
                self.nodes[n].parent = curr_node
                # rhs of successor = g of parent + path cost
                self.nodes[n].rhs = self.nodes[curr_node].g_cost + self.nodes[curr_node].neighbours[n]
                if not self.node_is_consistent(n):
                    self.open_list[n] = self.get_key(n, start_node)
                    self.open_length += 1
            curr_node = self.get_smallest(self.open_list)
        print("NNNNNNNNNNN len of nodes in open list:", len(self.open_list))
        current_path = []
        while not self.nodes[curr_node].parent == None:
            print("outside the 2nd while loop")
            # bg[505 - curr_node[1], curr_node[0] + 555] = (250, 0, 0)
               # Optional: visualize closed nodes
            # grid_viz.set_color(curr_node,"pale yellow")
            current_path.append(curr_node)
            curr_node = self.nodes[curr_node].parent
        current_path.append(curr_node)
        # cv2.imshow("D star lite output", bg)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print("Path length: ",len(current_path))
        # print("Path gen by Dstar lite:", current_path)
        return current_path

    # new neighbours
    def new_calculate_neighbours(self, curr_node, open_list, current_path):
        x = curr_node[0]
        y = curr_node[1]
        dig = 1.41
        strght = 1
        if (x-1,y+1) not in self.obstacle_space and x-1 >= -304 and y+1 < 192:
            if (x-1,y+1) not in open_list and not self.nodes[(x,y)].parent == (x-1,y+1) and (x-1,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x-1,y+1)] = dig
        if (x,y+1) not in self.obstacle_space and y+1 < 192:
            if (x,y+1) not in open_list and not self.nodes[(x,y)].parent == (x,y+1) and (x,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x,y+1)] = strght
        if (x+1,y+1) not in self.obstacle_space and x+1 < 304 and y+1 < 192:
            if (x+1,y+1) not in open_list and not self.nodes[(x,y)].parent == (x+1,y+1) and (x+1,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x+1,y+1)] = dig
        if (x-1,y-1) not in self.obstacle_space and x-1 >= -304 and y-1 >= -192:
            if (x-1, y-1) not in open_list and not self.nodes[(x,y)].parent == (x-1,y-1) and (x-1,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x-1, y-1)] = dig
        if (x,y-1) not in self.obstacle_space and y-1 >= -192:
            if (x, y-1) not in open_list and not self.nodes[(x,y)].parent == (x,y-1) and (x,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x, y-1)] = strght
        if (x+1,y-1) not in self.obstacle_space and x+1 < 304 and y-1 >= -192:
            if (x+1,y-1) not in open_list and not self.nodes[(x,y)].parent == (x+1,y-1) and (x+1,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x+1, y-1)] = dig
        if (x-1,y) not in self.obstacle_space and x-1 >= -304:
            if (x-1,y) not in open_list and not self.nodes[(x,y)].parent == (x-1,y) and (x-1,y) not in current_path:
                self.nodes[(x, y)].neighbours[(x-1,y)] = strght
        if (x+1,y) not in self.obstacle_space and x+1 < 304:
            if (x+1,y) not in open_list and not self.nodes[(x,y)].parent == (x+1,y) and (x+1,y) not in current_path:
                self.nodes[(x,y)].neighbours[(x+1,y)] = strght
    # get new key for replanning
    def get_new_key(self, node, start):
        key = min(self.nodes[node].g_cost, self.nodes[node].rhs) + self.h(node, start)
        return key


    
    def map_sub_callback(self, mmsg):
      print("calling the map callback function")

      
      # condition to check whether the subscriber is still active before trying to unregister it. Additionally, 
      # added a condition to only call the generate_goals method once, when the goals array is empty.
      if self.map_subscriber and self.callback_check == False:
        print("map read, now storing the data")
        self.callback_check = True
        self.map_data = mmsg
        self.map_subscriber.unregister()
        # self.map_subscriber.unregister()
        # print(self.map_data)

    def map_sub_initialisation(self):
      self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_sub_callback)

    def obstacle_check(self, x, y):
    #   self.obstacles_1d = []

    # #   print(self.map_data)

      if self.map_data is None:
        print("No map data available- cannot perform obstacle check")
        return False
    #   for idx, val in enumerate(self.map_data):
    #       if val == 100:
    #           self.obstacles_1d.append(idx)
    #   print("obstacle 2d")
    #   width_2d = self.map_data.info.width
    #   height_2d = self.map_data.info.height
    #   data_2d = self.map_data.data
    #   print("width_2d:", width_2d)
    #   print("height_2d:", height_2d)

    # # Reshape the 1D array into a 2D array
    #   grid = [data_2d[i:i+width_2d] for i in range(0, len(data_2d), width_2d)]

    # Now 'grid' is a 2D representation of the map
    #   print(grid)
   
      map_width = self.map_data.info.width
      map_resolution = self.map_data.info.resolution
      map_origin_x = self.map_data.info.origin.position.x
      map_origin_y = self.map_data.info.origin.position.y
      map_data_array = self.map_data.data

    #   # Check if the map index is an obstacle
    #   map_x = int((x - map_origin_x) / map_resolution)
    #   map_y = int((y - map_origin_y) / map_resolution)

    #   # Check if the map index is an obstacle
    #   print("map_x:", map_x)
    #   print("map_y:", map_y)
    #   print("cal:", map_y * map_width + map_x)
    #   print("length:", len(map_data_array))
      if map_data_array[map_width * x + y] == 100:
        # print("######################the genrated goal index has an obstacle")
        return True
      else:
        # print("doesnot belong to obstacle, its a free space")
        return False

    def send_goal(self, x, y):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id= "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x  = x
        goal.target_pose.pose.position.y  = y
        goal.target_pose.pose.orientation.w = 1.0 # No rotation of the mobile base frame w.r.t. map frame
        
        # Sends the goal to the action server.
        client.send_goal(goal)
    # Waits for the server to finish performing the action.
        wait_4_result = client.wait_for_result()
        print(wait_4_result)

    # If the result doesn't arrive, assume the Server is not available
        if not wait_4_result:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!---> shutdown")
        else:
            return client.get_result()   #Result of executing the action
            # self.get_goal_result()    


def initiate_Dstar_lite(start_tuple_x, start_tuple_y,goal_x, goal_y, height, width, costmap, resolution, origin, viz):
    global grid_viz, x, y, x_g, y_g, graph
    # rospy.init_node('D_start_lite', log_level=rospy.INFO, anonymous=False)
    graph = Graph()
    graph.map_sub_initialisation()
    rospy.sleep(2)
    costmap = costmap
    resolution =resolution
    origin =origin
    grid_viz = viz
    x_r = start_tuple_x
    y_r = start_tuple_y
    x_g = goal_x
    y_g = goal_y
    #global x
    #global y
    x = x_r
    y = y_r
    # graph.send_goal(x_g, y_g)
    print("Goal",x_g,y_g)
    print("received Height by Dstar-lite:", height)
    print("received width by Dstar-Lite:", width)
    # scale the window size
    height = 384
    width = 608
    # graph.obstacle_check(x, y)
    for x in range(-192, 192):
        for y in range(-304, 304):
            # y = -y
            if graph.obstacle_check(x, y):
                graph.obstacle_space.add((x, y))
    # print("obstacle_space:", graph.obstacle_space)
    graph.create_nodes()
    current_path = graph.d_star_lite_algo(x_r,y_r,x_g,y_g)
    # path =[]
    # for x, y in current_path:
    #     index = y * width + x
    #     path.append(index)
    #     return path
    return current_path
# replanning the path from current start point
def replan_dstar(rob_x, rob_y, goal_x, goal_y,current_path, dist_safety, time_cost):
    safety = dist_safety
    print("detected a dynamic obstacle, so replanning the path from the path")
    # graph2 = Graph()
    # get coordinates for the start node
    start_node = (rob_x, rob_y)
    # get coordinates for the goal node
    goal_node = (goal_x, goal_y)
    new_open_list = {}
    visited = []
    # make cost of start node zero
    graph.nodes[goal_node].rhs = 0
    new_open_list[goal_node] = 0  # key needs to be written here
    curr_node = goal_node
    parent = graph.nodes[goal_node].parent
    while not parent == None:
        visited.append(parent)
        parent = graph.nodes[parent].parent
    while not curr_node == start_node and not len(new_open_list) == 0:
        visited.append(curr_node)
        # make g_cost = rhs
        graph.nodes[curr_node].g_cost = graph.nodes[curr_node].rhs 
        graph.nodes[curr_node].g_cost = graph.nodes[curr_node].rhs + safety 
        # first make all neighbour rhs = infinitys
        # Now recalculate for new path
        graph.nodes[curr_node].neighbours = {}
        graph.new_calculate_neighbours(curr_node, new_open_list, visited)
        for n in graph.nodes[curr_node].neighbours:
            if n in visited:
                continue
            graph.nodes[n].parent = curr_node
            # rhs of successor = g of parent + path cost
            graph.nodes[n].rhs = graph.nodes[curr_node].g_cost + graph.nodes[curr_node].neighbours[n]
            if not graph.node_is_consistent(n):
                new_open_list[n] = graph.get_new_key(n, start_node)
        # remove curr_node from the open list
        del new_open_list[curr_node]
        curr_node = graph.get_smallest(new_open_list)
    new_path = []
    count = 0
    while not graph.nodes[curr_node].parent == None:
        new_path.append(curr_node)
        curr_node = graph.nodes[curr_node].parent
        count += 1
    new_path.append(curr_node)
    return new_path
