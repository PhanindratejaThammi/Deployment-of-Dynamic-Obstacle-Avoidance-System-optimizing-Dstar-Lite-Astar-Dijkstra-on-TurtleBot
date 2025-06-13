#!/usr/bin/env python

import time
import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz
from algorithms.astar import astar
import numpy as np
from numpy import savetxt
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import csv
from rosgraph_msgs.msg import Clock
import pandas as pd
from move_base_msgs.msg import MoveBaseActionResult
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
import threading
from std_msgs.msg import Bool
import os
from std_msgs.msg import Int32
from algorithms.Dstar_lite import initiate_Dstar_lite
from nav_msgs.msg import OccupancyGrid
from algorithms.Dstar_lite import replan_dstar



def make_plan(req):
  global start_index, goal_index, path, start_time_1, previous_plan_variables, goal_x, goal_y, rec_path, resp
  resp = None 
  # rec_path = None
  start_time_1 =  rospy.Time.now()
  print("Start time",start_time_1)

    # costmap as 1-D array representation
  costmap = req.costmap_ros
    # number of columns in the occupancy grid
  width = req.width
    # number of rows in the occupancy grid
  height = req.height
  start_index = req.start
  goal_index = req.goal
  print("initial index:", start_index )
  print("goal point index", goal_index )
  start_tuple_x = start_index % width
  start_tuple_y = int(start_index / width)
    #deploying the contract
  goal_x = goal_index % width
  goal_y = int(goal_index / width)
  print("start point:", [(start_tuple_x, start_tuple_y)])
  print("goal point:", [(goal_x, goal_y)])

    # side of each grid map square in meters
  resolution = 0.05
    # origin of grid map
  origin = [-100, -100, 0]
  viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)

    # time statistics
  start_time = rospy.Time.now()
    # calculate the shortes path
  path, previous_plan_variables = astar(start_index, goal_index, width, height, costmap, resolution, origin, viz, previous_plan_variables)
  # rec_path = initiate_Dstar_lite(start_tuple_x, start_tuple_y,goal_x, goal_y, height, width, costmap, resolution, origin, viz)
  print(" Path from algorithm in coordinates",path)
  
  ####____uncomment this incase of the D-star lite____####
  # print(" Path from algorithm in coordinates",rec_path)
  # path =[]
  # for x, y in rec_path:
  #   index = y * width + x
  #   path.append(index)
  #     # path=[]
  print("length of the path:", len(path))
  if not path:
    rospy.logwarn("No path returned by the path algorithm")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('++++++++ Path Planning execution metrics ++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('Path sent to navigation stack')
    

    resp = PathPlanningPluginResponse()
    resp.plan = path
    

    if path is not None:
     robot_coord.path_reader()
  return resp
    

#defining class regarding all the data of robot coordinates and its storage.
class Coordinates_storage:
  global start_index, global_path_x_coordinate, global_path_y_coordinate, time_note_con2, goal_x_coordinate, goal_y_coordinate

  def __init__(self):
      self.time_note_con1 = False
      self.initial_con1 = False
      self.value_goal_confirm = False
      self.goal_coordinates_array = []
      self.initial_coordinates_array = []
      self.global_path_points_array = []
      self.arm_count_to_Goal = 0

      # Publisher to publish the value of self.value_goal_confirm
      self.goal_confirm_pub = rospy.Publisher('goal_confirm_topic', Bool, queue_size=1)
    

  def goal_receiver_callback(self, pgoal):
    # Access the goal information
    pose = pgoal.target_pose.pose
    x = pose.position.x
    y = pose.position.y
    theta = pose.orientation.w


  def goal_receiver(self):
   rospy.Subscriber('/goal_receiverr', MoveBaseGoal, self.goal_receiver_callback)

 
  def sum_weights(self, path):
     self.cumu_add_weights = path
     print("$$$$$$$$$$$$$$$$$$$$$$$ goal coordniates array inside sum_weights funtion:",path)
     sum = 0
     for i in path:
        sum = sum + i
     print("********************sum of the weights of the path followed by the robot:************************", sum)
     return sum
  
  def tot_sim_time(self,start_time_1,end_time_1):
     global final_sim_time
     final_sim_time = end_time_1 - start_time_1 
     print(" Total simulation time of the robot(nano_secs):", final_sim_time)
     final_sim_time_secs = final_sim_time.to_sec()
     print(" Total simulation time of the robot(secs):", final_sim_time_secs)

     return final_sim_time_secs

  
  def goal_pose_callback(self, msg):
      global goal_x_coordinate, goal_y_coordinate, goal_coordinates
      print("**********************code started executing goal_pose_callback function ********************8")
      goal_coordinates = (msg.goal.target_pose.pose.position.x, msg.goal.target_pose.pose.position.y)
      goal_x_coordinate = (msg.goal.target_pose.pose.position.x)
      goal_y_coordinate = (msg.goal.target_pose.pose.position.y)
      self.goal_coordinates_array.append(goal_coordinates)
      rospy.loginfo("printing x_goalpoint coordinate: %s", goal_x_coordinate)
      rospy.loginfo("printing y_goal point coordniate: %s", goal_y_coordinate)
      print('newly assigned goalpoint coordinates are:', goal_coordinates)
      print('Total goalpoint coordiantes(array) storage of robot:', self.goal_coordinates_array)


  def goal_pose_listener(self):
     rospy.loginfo("********goal pose listener initiated*********")
     self.goalpose = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_pose_callback)
     print("code is called callback funtion of goal pose listener")
  
  def initial_pose_callback(self, imsg):
      print("**********code started executing initlal_pose_callback fgoal_pose_listenerunction**********")
      global initial_x_coordinate, initial_y_coordinate
      initial_pose = imsg.pose.pose
      initial_coordinates = (initial_pose.position.x, initial_pose.position.y)
      initial_x_coordinate = (initial_pose.position.x)
      initial_y_coordinate = (initial_pose.position.y)
      self.initial_coordinates_array.append(initial_coordinates)
      print("priniting initial x coordinate", initial_x_coordinate)
      print("printing initial y coordinate", initial_y_coordinate)
      print("newly appended initial coordinates are:",initial_coordinates)
      print("Total inital coordiantes storage of robot:", self.initial_coordinates_array)
      self.initialpose.unregister()
  
  def initial_pose_listener(self):
    self.initialpose = rospy.Subscriber('/odom', Odometry, self.initial_pose_callback)
    print("code is called callback funtion of initial pose listener")

  def path_reader_callback(self, pmsg):
     path_point = pmsg.pose.pose.position
     path_x_coordinate = (path_point.x)
     path_y_coordinate = (path_point.y)
     self.time_reader(path_x_coordinate, path_y_coordinate, goal_x_coordinate, goal_y_coordinate)


  def time_reader(self, path_x_coordinate, path_y_coordinate, goal_x_coordinate, goal_y_coordinate):
   
     global end_time_1, count, time_note_con2
     simulation_time = None
     distance = None
     time_note_con2 = False
     if self.coordinates_match(path_x_coordinate, goal_x_coordinate) and self.coordinates_match(path_y_coordinate, goal_y_coordinate) and (time_note_con2==False) :
      end_time_1 = rospy.Time.now()
      print("printing the end time ", end_time_1)
      time_note_con2 = True
      self.path_read.unregister()
      simulation_time = self.tot_sim_time(start_time_1,end_time_1)
      print("final simulation time", simulation_time)
      sum_of_weights = self.sum_weights(path)
      distance = self.distance_cal(start_index,goal_index)
      print("final distance travelled ", distance)
      self.export_all_data_to_csv(goal_coordinates, start_index, goal_index, sum_of_weights, simulation_time, path, distance)
      self.value_goal_confirm = True
      # print("self.value_goal_confirm", self.value_goal_confirm)

     else:
      # value_time_reader = False  # Set the value of value_time_reader to False
      self.value_goal_confirm = False
      # print("self.value_goal_confirm", self.value_goal_confirm)

      #checking not only reaching the goal but also droped the object succesfully
     if self.value_goal_confirm is True:
        self.goal_confirm_pub.publish(self.value_goal_confirm)
     return simulation_time, distance


    #  return self.value_goal_confirm
    #  self.value_update_event.set() # Set the event to signal update
    
  def distance_cal(self, start_index, goal_index):
     total_distance = abs(goal_index - start_index)
     print("code just calculated the disatance")
     return total_distance

     
      #call the function to call the second nav goal
  def store_time_reader(self, value_time_reader):
     goal_reach_confirm = value_time_reader
     print("goal reach confirmation that will be passed to other script to gen new goal:", goal_reach_confirm)
     
     

  ## defined function with tolerace for matching  
  def coordinates_match(self, arg1, arg2):
    # print("printing the value of the arg1", arg1)
    # print("printing the values of the arg2", arg2)
    if abs(arg1 - arg2) < 0.2 and abs(arg1 - arg2) < 0.2:
     return True
  def path_reader(self):  
     self.path_read = rospy.Subscriber('/odom', Odometry, self.path_reader_callback)
     print("code started calling the path_reader callback")

  def global_plan_path_callback(self, pmsg):
     global global_path_x_coordinate, global_path_y_coordinate
     first_path_point = pmsg.poses[0].pose.position
     global_path_point_coordinates = (first_path_point.x, first_path_point.y)
     global_path_x_coordinate = (first_path_point.x)
     global_path_y_coordinate = (first_path_point.y)
     
     if self.initial_con1 == False:
        self.global_path_points_array.append(global_path_point_coordinates)
        print("printing the values of robot rest position, global path topic-x_coordinate, initial coordinates:", global_path_x_coordinate)
        print("printing the values of robot rest position, global path topic-y_coordinate, initial coordinates:", global_path_y_coordinate)
        print('Printing the values of robot upated initial coordiantes(array) storage of robot:', self.global_path_points_array)
        self.initial_con1 = True

  def global_plan_path(self):
     self.robo_path = rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, self.global_plan_path_callback)
     
     
  def clock_callback(self, clock_msg):
     print("clock callback function initiated")
    #  self.start_clc_time = rospy.Time.now()
     self.current_time = clock_msg.clock.secs
     rospy.loginfo("Current time published in /clock topic : %s", self.current_time)
    #  rospy.loginfo = ("publishing the time data using rospy.time.now:", self.start_clc_time )
    #  self.clock_data = self.current_time
     self.clock_sub.unregister() # Unsubscribe from the topic after receiving the first message
     
      

  def clock_listener(self):
    print("code started to execute the clock_listener()")
    self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)
    rospy.loginfo("callback function is about to start, Clock initiated and started noting simulation time")
  
  
  def generate_goals(self, x_coord_range, y_coord_range, num_goals):  # (num_goals: how many Generate random number of  goal coordinates within the desired range
    # rate = rospy.Rate(1/interval) #setting the publishing rate: 1 goal for every interval, in this case:10 secs
    goals = [] #creating the array with all the goal randomly generateed goals in a range mentioned
    for i in range(num_goals):
       x = np.random.uniform(2, x_coord_range) #result is a list of 1 value; in range x_coord_range, y_coord_range with no repeats 
       y = np.random.uniform(2, y_coord_range) #result is a list of 1 value; x_coord_range, y_coord_range with  no repeats.
       print("printing the random goal x-coordinate:", x)
       print("printing the random goal x-coordinate:", y)
       # Creates a new goal with the MoveBaseGoal constructor
       goal = MoveBaseGoal()
       goal.target_pose.header.frame_id= "map"
       goal.target_pose.header.stamp = rospy.Time.now()
       goal.target_pose.pose.position.x  = x
       goal.target_pose.pose.position.y  = y
       goal.target_pose.pose.orientation.w = 1 # No rotation of the mobile base frame w.r.t. map frame
       goals.append(goal)
         # Sends the goal to the action server.
       client.send_goal(goals[0])
   # Waits for the server to finish performing the action.
       wait = client.wait_for_result()
    



   # If the result doesn't arrive, assume the Server is not available
       if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
       else:#Result of executing the action
          return client.get_result()   
    return goals
    
  
  def conditions_goal(self, result_status):
      print("printing current icon value:", icon)
      print("printing the current status.status value in /move_base/result topic", result_status)
      if icon == 0 and result_status != 3 and self.current_count < len(self.goals):
       print("Assigning the first goal to the robot")
       self.send_goal(self.goals[self.current_count])
      #  icon = icon +1
      # elif icon != 0 and result_status == 3 and self.current_count < len(self.goals):
      #   self.current_count = self.current_count + 1
      #   print("The value of current_count is updated and assinging the next consecutive goal point in the array to the robot")
      #   self.send_goal(self.goals[self.current_count])
      # else:
      #   print("Either no goals are available or all the goals are reached, navigation completed")
     
  
  def goal_initialization_callback(self, result):
     global result_status
     print("code entered the intialisation callback function")
     result_status = result.status.status
     print("printing the value of  result.status.status", result_status)
     self.conditions_goal()

  def send_pub_goal(self, x, y):
     goal = PoseStamped()
     goal.header.frame_id = "map"
     goal.pose.position.x = x
     goal.pose.position.y = y
     goal.pose.orientation.w = 1.0  # Set a default orientation
     print("printing the value of selected  coordinates:", goal.pose.position.x)
     print("printing the value of selected  coordinates:", goal.pose.position.y)
     print("printing the postamped value in publishing function", goal)
     self.goal_pub.publish(goal) 
       
  def goal_pub_check(self):
     print("code initialised the goal_pub_check.")
    #  print("printing the ")
     self.goal_pub.publish(self.goals[0])

  def goal_pub_initialize(self, x_coord_range, y_coord_range):
    x = np.random.uniform(2, x_coord_range) #result is a list of 1 value; in range x_coord_range, y_coord_range with no repeats 
    y = np.random.uniform(2, y_coord_range) 
    self.send_pub_goal(x, y)
    
        
  def goal_intialization(self):
     print("code entered to initalize the goal coordinates to the robot")
    #  self.current_count = 0
    #  print("printing the array with the goal coordinates", self.goals)
     self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2)
    #  self.goals = self.generate_goals(7,7,2)#generating a list of 2 goals
    #  self.rate = rospy.Rate(0.1)  # Control the rate at which goals are sent (1 goal every 20 seconds) 
    #  rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_initialization_callback)
    #  print("printing the array with the goal coordinates", self.goals[0])
    #  self.goal_pub.publish(self.goals[0]) 

    #  print("goal generating is done and calling the callback function")
        

    


# Creating structure for the CSV File
  def export_all_data_to_csv(self, goal_coordinates, start_index, goal_index, sum_1, final_sim_time_secs, gen_path, dist_ance):

    print("*********code entered for exporting files*****************")
    print("goal coordinates in csv file :", goal_coordinates)
    print("printing strting indices inside csv:", start_index)
    print("printing goal indices inside csv", goal_index)
    print("printing cumulative weights sum:", sum_1)
    print("Robot simulation time in secs:", final_sim_time_secs)
    print("Generated path for the robot:", path)
    print("Printing the distance trvelled by robot:", dist_ance)


    # headerList = ['Goal Coordinates', 'Initial Index(wt)', 'Goal Index(wt)', 'Sum of weights', 'total sim time(secs)']
    
    data = {'Goal Coordinates':[goal_coordinates],
            'Initial Index(wt)':[start_index],
            'Goal Index(wt)':[goal_index],
            'Sum of weights':[sum_1],
            'total sim time(secs)':[final_sim_time_secs],
            'path indices':[gen_path],
            'distance':[dist_ance],
            }
    # data = {[goal_coordinates],
    #         [start_index],
    #         [goal_index],
    #         [sum],
    #         [final_sim_time_secs]
    #         }
    df = pd.DataFrame(data)
    # Check if the CSV file exists
    filename = '/home/ubuntu-smart-msc/block_chain_proj_sanjaya/BlockchainRL_BaseVersion/BlockchainRL_BaseVersion/turtlebot3_ws/src/global_path_planning/scripts/Robot_data_collection.csv'
    file_exists = os.path.isfile(filename)

    # Append data to CSV
    with open(filename, mode='a') as csvfile:
        df.to_csv(csvfile, index=False, header=not file_exists)


    
    # df.to_csv('/home/ubuntu-smart-msc/block_chain_proj_sanjaya/BlockchainRL_BaseVersion/BlockchainRL_BaseVersion/turtlebot3_ws/src/global_path_planning/scripts/Robot_data_collection.csv', mode = 'a', index=False, header = True)
    # # a = df.pivot('Goal Coordniates')
    print("printing data frames using pandas", df)         
    
def send_goal(self, x, y):
    # x = np.random.uniform(2, x_coord_range) #result is a list of 1 value; in range x_coord_range, y_coord_range with no repeats 
    # y = np.random.uniform(2, y_coord_range) #result is a list of 1 value; x_coord_range, y_coord_range with  no repeats.
    # print("printing the random goal x-coordinate:", x)
    # print("printing the random goal x-coordinate:", y)
       # Creates a new goal with the MoveBaseGoal constructor
       goal = MoveBaseGoal()
       goal.target_pose.header.frame_id= "map"
       goal.target_pose.header.stamp = rospy.Time.now()
       goal.target_pose.pose.position.x  = x
       goal.target_pose.pose.position.y  = y
       goal.target_pose.pose.orientation.w = 1.0 # No rotation of the mobile base frame w.r.t. map frame
      #  goals.append(goal)ss
         # Sends the goal to the action server.
       client.send_goal(goal)
   # Waits for the server to finish performing the action.
       wait = client.wait_for_result()

   # If the result doesn't arrive, assume the Server is not available
       if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
       else:#Result of executing the action
          return client.get_result()  


     
 


   
def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)


class replan:
   def __init__(self):
      global rec_path
      self.new_path = []
      self.turtlepose = rospy.Subscriber('/odom', Odometry, self.turtle_pose_callback)


   def turtle_pose_callback(self, odom_data):
      self.turtle_pose = odom_data.pose.pose
      self.turtle_coordinates = (self.turtle_pose.position.x, self.turtle_pose.position.y)
      self.turtle_x_coordinate = (self.turtle_pose.position.x)
      self.turtle_y_coordinate = (self.turtle_pose.position.y)
      self.turtlepose.unregister()
     

   def map_callback(self, data):
      # Get the information about the map
      global rec_path      
      map_width = data.info.width
      resolution = data.info.resolution
      origin_x = data.info.origin.position.x
      origin_y = data.info.origin.position.y
      # Assuming the robot's next index is stored in some variable, replace this with your actual logic
        # Calculate the index in the map array
    #   map_index = next_index_y * width + next_index_x
  
        # Check the value in the map at the calculated index
      print("rec_path:", rec_path)
      if rec_path is not None:
          print("Initial path is already received")
          
          # Initialize an index to keep track of the current position in the path
          current_index = 0   
          while current_index < len(rec_path):
              r = rec_path[current_index]
              # print("Selected coord in the path:", r)
              r_x, r_y = r
              current_index += 1
              # Check for a significant change in robot position
              # if (abs(r_x - self.turtle_x_coordinate) >= 1.0 and abs(r_y - self.turtle_y_coordinate) >= 1.0):
                  # print("Significant change in robot position detected")
                   # Check if there is a next element in the path
              if current_index < len(rec_path):
                  nr_x, nr_y = rec_path[current_index]
                  # print("look ahead point:", (nr_x, nr_y))
                  # print("value at look ahead point:", data.data[map_width * nr_x + nr_y])
                  # Assuming data is available and you want to check for dynamic obstacles
                  if data.data[map_width * nr_x + nr_y] == 100:
                      print("values of updated map:",data.data[map_width * nr_x + nr_y])
                      print("$$$$$$$$$$$$$$$ I'm seeing a dynamic obstacle")
                      dist_from_obs = data.data[map_width * nr_x + nr_y] + 2
                      time_obs = rospy.Time()
                      # Call the replan function and update the path
                      self.new_path = replan_dstar(r_x, r_y, goal_x, goal_y, rec_path, dist_from_obs, time_obs)
                      print("Replanning is done, providing the new path")
                      resp.plan = self.new_path  # assign the new path to the path planning plugin
                      # Update the current index to continue checking the new path
                      current_index = 0
                      rec_path = self.new_path
        # No significant change or obstacle detected, move to the next index
          


      else: 
         print("initial path is not calculated yet")

      if self.new_path is None:
         print("new path is not generated")
            # self.path_reader()
   

if __name__ == '__main__':

  global start_time_1, end_time_1, client
  # count=0
  icon =0
  rec_path = None
  previous_plan_variables = None

  print("****************main function started executing*********")
  rospy.init_node('service_server_RL', log_level=rospy.INFO, anonymous=False)
  robot_coord = Coordinates_storage()
  replan_fxn = replan()
  rospy.sleep(1)

  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  rospy.sleep(2)
  print("################Code is about to execute the store_coordinates to print initial and goal coordinates#########")

  robot_coord.goal_pose_listener()
  # if rec_path is not None:
  # map_sub = rospy.Subscriber('/map', OccupancyGrid,replan_fxn.map_callback)
  
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  
  rospy.on_shutdown(clean_shutdown)
  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
  